from copy import deepcopy
import os
import pickle
import random
import itertools
import numpy as np
import torch
from torch.optim import Adam
import gym
import time
import rospy
import spinup.algos.pytorch.sac.core as core
from spinup.utils.logx import EpochLogger


class ReplayBuffer:
  """A simple FIFO experience replay buffer for SAC agents."""

  def __init__(self, obs_dim, act_dim, size):
    self.obs_buf = np.zeros(core.combined_shape(size, obs_dim), dtype=np.float32)
    self.obs2_buf = np.zeros(core.combined_shape(size, obs_dim), dtype=np.float32)
    self.act_buf = np.zeros(core.combined_shape(size, act_dim), dtype=np.float32)
    self.rew_buf = np.zeros(size, dtype=np.float32)
    self.done_buf = np.zeros(size, dtype=np.float32)
    self.ptr, self.size, self.max_size = 0, 0, size

  def store(self, obs, act, rew, next_obs, done):
    self.obs_buf[self.ptr] = obs
    self.obs2_buf[self.ptr] = next_obs
    self.act_buf[self.ptr] = act
    self.rew_buf[self.ptr] = rew
    self.done_buf[self.ptr] = done
    self.ptr = (self.ptr+1) % self.max_size
    self.size = min(self.size+1, self.max_size)

  def sample_batch(self, batch_size=32):
    idxs = np.random.randint(0, self.size, size=batch_size)
    batch = dict(obs=self.obs_buf[idxs],
                  obs2=self.obs2_buf[idxs],
                  act=self.act_buf[idxs],
                  rew=self.rew_buf[idxs],
                  done=self.done_buf[idxs])
    return {k: torch.as_tensor(v, dtype=torch.float32) for k,v in batch.items()}

  def get_transition_at(self, pos):
    return self.obs_buf[pos], self.act_buf[pos], self.rew_buf[pos], self.obs2_buf[pos], self.done_buf[pos]


def save_model_training(path, epoch, ac, ac_targ, replay_buffer, pi_optimizer, q_optimizer, t_total):
  """Save the model and all things necessary for continueing the training.
  
  Args:
    path (str):     Path to the directory of checkpoints to save
    epoch (int):    Current epoch number
    ac:             Actor-critic model
    ac_targ:        Target actor-critic model
    replay_buffer:  Buffer that contains all transitions
    pi_optimizer:   Adam optimizer for policy
    q_optimizer:    Adam optimizer for q-value function approximator
    t_total (int):  Total number of timesteps
  """
  checkpoint = {
    'epoch': epoch,
    'state_dict': ac.state_dict(),
    'target_state_dict': ac_targ.state_dict(),
    'pi_optimizer': pi_optimizer.state_dict(),
    'q_optimizer': q_optimizer.state_dict(),
    't_total': t_total
  }
  f_path = path + '/checkpoint_' + str(epoch) + '.pt'
  torch.save(checkpoint, f_path)
  rb_path = path + '/replay_buffer.pkl'
  with open(rb_path, 'wb') as outp:
    pickle.dump(replay_buffer, outp, pickle.HIGHEST_PROTOCOL)
    

def load_model_training(path, epoch, ac, ac_targ, pi_optimizer, q_optimizer):
  """Load the model and all things necessary for continueing the training.
  
  Args:
    path (str):     Path to the directory of checkpoints to save
    epoch (int):    Current epoch number
    ac:             Actor-critic model
    ac_targ:        Target actor-critic model
    pi_optimizer:   Adam optimizer for policy
    q_optimizer:    Adam optimizer for q-value function approximator
  
  Returns:
    ac:             Actor-critic model
    ac_targ:        Target actor-critic model
    replay_buffer:  Buffer that contains all transitions
    pi_optimizer:   Adam optimizer for policy
    q_optimizer:    Adam optimizer for q-value function approximator
    t_total (int):  Total number of timesteps
  """
  checkpoint_path = path + '/checkpoint_' + str(epoch) + '.pt'
  checkpoint = torch.load(checkpoint_path)
  ac.load_state_dict(checkpoint['state_dict'])
  ac_targ.load_state_dict(checkpoint['target_state_dict'])
  pi_optimizer.load_state_dict(checkpoint['pi_optimizer'])
  q_optimizer.load_state_dict(checkpoint['q_optimizer'])
  # replay buffer
  rb_path = path + '/replay_buffer.pkl'
  with open(rb_path, 'rb') as inp:
    replay_buffer = pickle.load(inp)
  return ac, ac_targ, replay_buffer, pi_optimizer, q_optimizer, checkpoint['t_total']


def sac_her(env, test_env, actor_critic=core.MLPActorCritic, ac_kwargs=dict(), seed=0, 
        n_epochs = 100, n_episodes_per_epoch = 20, replay_size=int(1e6), gamma=0.99, 
        polyak=0.995, lr=1e-3, alpha=0.2, batch_size=100, start_steps=10000, 
        update_after=1000, update_every=50, num_test_episodes=10, max_ep_len=20, 
        n_updates = 40, k_her_samples = 4, logger_kwargs=dict(), save_freq=1, load_epoch=-1):
  """
  Soft Actor-Critic (SAC) with Hindsight Experience Replay


  Args:
      env: The environment must satisfy the OpenAI Gym API.

      test_env: The test environment must satisfy the OpenAI Gym API.
                If this is None, tests are not performed.

      actor_critic: The constructor method for a PyTorch Module with an ``act`` 
          method, a ``pi`` module, a ``q1`` module, and a ``q2`` module.
          The ``act`` method and ``pi`` module should accept batches of 
          observations as inputs, and ``q1`` and ``q2`` should accept a batch 
          of observations and a batch of actions as inputs. When called, 
          ``act``, ``q1``, and ``q2`` should return:

          ===========  ================  ======================================
          Call         Output Shape      Description
          ===========  ================  ======================================
          ``act``      (batch, act_dim)  | Numpy array of actions for each 
                                          | observation.
          ``q1``       (batch,)          | Tensor containing one current estimate
                                          | of Q* for the provided observations
                                          | and actions. (Critical: make sure to
                                          | flatten this!)
          ``q2``       (batch,)          | Tensor containing the other current 
                                          | estimate of Q* for the provided observations
                                          | and actions. (Critical: make sure to
                                          | flatten this!)
          ===========  ================  ======================================

          Calling ``pi`` should return:

          ===========  ================  ======================================
          Symbol       Shape             Description
          ===========  ================  ======================================
          ``a``        (batch, act_dim)  | Tensor containing actions from policy
                                          | given observations.
          ``logp_pi``  (batch,)          | Tensor containing log probabilities of
                                          | actions in ``a``. Importantly: gradients
                                          | should be able to flow back into ``a``.
          ===========  ================  ======================================

      ac_kwargs (dict): Any kwargs appropriate for the ActorCritic object 
          you provided to SAC.

      seed (int): Seed for random number generators.

      steps_per_epoch (int): Number of steps of interaction (state-action pairs) 
          for the agent and the environment in each epoch.

      n_epochs (int): Number of epochs to run and train agent.

      n_episodes_per_epoch (int): Number of episodes per epoch

      replay_size (int): Maximum length of replay buffer.

      gamma (float): Discount factor. (Always between 0 and 1.)

      polyak (float): Interpolation factor in polyak averaging for target 
          networks. Target networks are updated towards main networks 
          according to:

          .. math:: \\theta_{\\text{targ}} \\leftarrow 
              \\rho \\theta_{\\text{targ}} + (1-\\rho) \\theta

          where :math:`\\rho` is polyak. (Always between 0 and 1, usually 
          close to 1.)

      lr (float): Learning rate (used for both policy and value learning).

      alpha (float): Entropy regularization coefficient. (Equivalent to 
          inverse of reward scale in the original SAC paper.)

      batch_size (int): Minibatch size for SGD.

      start_steps (int): Number of steps for uniform-random action selection,
          before running real policy. Helps exploration.

      update_after (int): Number of env interactions to collect before
          starting to do gradient descent updates. Ensures replay buffer
          is full enough for useful updates.

      update_every (int): Number of env interactions that should elapse
          between gradient descent updates. Note: Regardless of how long 
          you wait between updates, the ratio of env steps to gradient steps 
          is locked to 1.

      num_test_episodes (int): Number of episodes to test the deterministic
          policy at the end of each epoch.

      max_ep_len (int): Maximum length of trajectory / episode / rollout.

      n_updates (int): Number of update steps at each update

      k_her_samples (int): Number of additional HER transitions per real transition

      logger_kwargs (dict): Keyword args for EpochLogger.

      save_freq (int): How often (in terms of gap between epochs) to save
          the current policy and value function.

      load_epoch (int): If -1, don't load. If >= 0, load the given epoch checkpoint and continue training there.

  """

  logger = EpochLogger(**logger_kwargs)
  logger.save_config(locals())

  torch.manual_seed(seed)
  np.random.seed(seed)

  perform_test = True
  if test_env is None:
    perform_test = False

  obs_dim = env.observation_space.shape
  act_dim = env.action_space.shape[0]

  # Action limit for clamping: critically, assumes all dimensions share the same bound!
  act_limit = env.action_space.high[0]

  # Create actor-critic module and target networks
  ac = actor_critic(env.observation_space, env.action_space, **ac_kwargs)
  ac_targ = deepcopy(ac)

  # Freeze target networks with respect to optimizers (only update via polyak averaging)
  for p in ac_targ.parameters():
      p.requires_grad = False
      
  # List of parameters for both Q-networks (save this for convenience)
  q_params = itertools.chain(ac.q1.parameters(), ac.q2.parameters())

  # Experience buffer
  replay_buffer = ReplayBuffer(obs_dim=obs_dim, act_dim=act_dim, size=replay_size)

  # Count variables (protip: try to get a feel for how different size networks behave!)
  var_counts = tuple(core.count_vars(module) for module in [ac.pi, ac.q1, ac.q2])
  logger.log('\nNumber of parameters: \t pi: %d, \t q1: %d, \t q2: %d\n'%var_counts)

  # Set up function for computing SAC Q-losses
  def compute_loss_q(data):
    o, a, r, o2, d = data['obs'], data['act'], data['rew'], data['obs2'], data['done']

    q1 = ac.q1(o,a)
    q2 = ac.q2(o,a)

    # Bellman backup for Q functions
    with torch.no_grad():
      # Target actions come from *current* policy
      a2, logp_a2 = ac.pi(o2)

      # Target Q-values
      q1_pi_targ = ac_targ.q1(o2, a2)
      q2_pi_targ = ac_targ.q2(o2, a2)
      q_pi_targ = torch.min(q1_pi_targ, q2_pi_targ)
      backup = r + gamma * (1 - d) * (q_pi_targ - alpha * logp_a2)

    # MSE loss against Bellman backup
    loss_q1 = ((q1 - backup)**2).mean()
    loss_q2 = ((q2 - backup)**2).mean()
    loss_q = loss_q1 + loss_q2

    # Useful info for logging
    q_info = dict(Q1Vals=q1.detach().numpy(),
                  Q2Vals=q2.detach().numpy())

    return loss_q, q_info

  # Set up function for computing SAC pi loss
  def compute_loss_pi(data):
    o = data['obs']
    pi, logp_pi = ac.pi(o)
    q1_pi = ac.q1(o, pi)
    q2_pi = ac.q2(o, pi)
    q_pi = torch.min(q1_pi, q2_pi)

    # Entropy-regularized policy loss
    loss_pi = (alpha * logp_pi - q_pi).mean()

    # Useful info for logging
    pi_info = dict(LogPi=logp_pi.detach().numpy())

    return loss_pi, pi_info

  # Set up optimizers for policy and q-function
  pi_optimizer = Adam(ac.pi.parameters(), lr=lr)
  q_optimizer = Adam(q_params, lr=lr)


  def update(data):
    # First run one gradient descent step for Q1 and Q2
    q_optimizer.zero_grad()
    loss_q, q_info = compute_loss_q(data)
    loss_q.backward()
    q_optimizer.step()

    # Record things
    logger.store(LossQ=loss_q.item(), **q_info)

    # Freeze Q-networks so you don't waste computational effort 
    # computing gradients for them during the policy learning step.
    for p in q_params:
      p.requires_grad = False

    # Next run one gradient descent step for pi.
    pi_optimizer.zero_grad()
    loss_pi, pi_info = compute_loss_pi(data)
    loss_pi.backward()
    pi_optimizer.step()

    # Unfreeze Q-networks so you can optimize it at next DDPG step.
    for p in q_params:
      p.requires_grad = True

    # Record things
    logger.store(LossPi=loss_pi.item(), **pi_info)

    # Finally, update target networks by polyak averaging.
    with torch.no_grad():
      for p, p_targ in zip(ac.parameters(), ac_targ.parameters()):
        # NB: We use an in-place operations "mul_", "add_" to update target
        # params, as opposed to "mul" and "add", which would make new tensors.
        p_targ.data.mul_(polyak)
        p_targ.data.add_((1 - polyak) * p.data)

  def get_action(o, deterministic=False):
    return ac.act(torch.as_tensor(o, dtype=torch.float32), deterministic)

  def test_agent():
    for j in range(num_test_episodes):
      o, d, ep_ret, ep_len = test_env.reset(), False, 0, 0
      collided = False
      critically_collided = False
      goal_reached = False
      while not(d or (ep_len == max_ep_len)):
        # Take deterministic actions at test time 
        o, r, d, _ = test_env.step(get_action(o, True))
        ep_ret += r
        ep_len += 1
        if d:
          if env.env._get_collision_from_obs(o):
            collided = True
            if env.env._get_critical_collision_from_obs(o):
              critically_collided = True
          else:
            # Maybe not the cleanest check. #TODO improve this.
            goal_reached = True
      logger.store(TestEpRet=ep_ret, TestEpLen=ep_len, TestGoalReached=goal_reached, TestCollided=collided, 
          TestCriticallyCollided = critically_collided)
      rospy.loginfo("# /// Deterministic Test Reward /// => {}".format(ep_ret))

  # Prepare for interaction with environment
  start_time = time.time()
  highest_reward = -np.inf
  t_total = 0
  t_last_update = 0
  start_epoch = 0
  # Set up model saving
  checkpoint_dir = logger.output_dir + '/checkpoints'
  if load_epoch>=0:
    start_epoch = load_epoch
    ac, ac_targ, replay_buffer, pi_optimizer, q_optimizer, t_total = load_model_training(checkpoint_dir, load_epoch, ac, ac_targ, pi_optimizer, q_optimizer)
    t_last_update = t_total
  else:
    os.makedirs(checkpoint_dir)
    #logger.setup_pytorch_saver(ac)
  
  for epoch in range(start_epoch, n_epochs):
    for episode in range(n_episodes_per_epoch):
      ## Real episode
      # Initialize s_init and goal
      o = env.reset()
      ep_ret = 0
      d = False
      local_buffer = ReplayBuffer(obs_dim=obs_dim, act_dim=act_dim, size=max_ep_len)
      goal_reached = False
      collided = False
      critically_collided = False
      for t in range(max_ep_len):
        # Until start_steps have elapsed, randomly sample actions
        # from a uniform distribution for better exploration. Afterwards, 
        # use the learned policy. 
        if t_total > start_steps:
            a = get_action(o)
        else:
            a = env.action_space.sample()

        # Step the env. 
        o2, r, d, info = env.step(a)
        # The robot control is allowed to change the action. For example if the action would result in collision.
        if 'action' in info:
          a = info['action']
          
        ep_ret += r

        # Store experience to replay buffer
        local_buffer.store(o, a, r, o2, d)

        # Super critical, easy to overlook step: make sure to update 
        # most recent observation!
        o = o2

        # Increment t_total
        t_total += 1

        # If episode done: break episode
        if d:
          if env.env._get_collision_from_obs(o2):
            collided = True
            if env.env._get_critical_collision_from_obs(o2):
              critically_collided = True
          else:
            # Maybe not the cleanest check. #TODO improve this.
            goal_reached = True
          break
      
      ## End of episode
      logger.store(EpRet=ep_ret, EpLen=t, GoalReached=goal_reached, Collided=collided, 
          CriticallyCollided = critically_collided)
      if collided:
        rospy.logerr("EPISODE FINISHED BY COLLISION")
      elif goal_reached:
        rospy.loginfo("EPISODE FINISHED BY REACHING THE GOAL")
      else:
        rospy.logwarn("EPISODE FINISHED BY TIMEOUT")
      rospy.loginfo("# FINAL episode length => {}".format(t))
      rospy.loginfo("# FINAL episode cumulated reward => {}".format(ep_ret))
      highest_reward = max(highest_reward, ep_ret)
      rospy.loginfo("# Highest reward yet => {}".format(highest_reward))
      ## HER
      # Add real and virtual transitions to replay buffer
      for t_replay in range(t):
        # Add real transition to experience buffer
        o_t, a_t, r_t, o2_t, d_t = local_buffer.get_transition_at(t_replay)
        replay_buffer.store(o_t, a_t, r_t, o2_t, d_t)
        # HER needs a future transition
        if (t_replay == t):
          break
        # Add virtual episode to experience buffer
        # Choose k future transitions as new goals
        remaining_ids = range(t_replay+1, t-1)
        if len(remaining_ids) >= k_her_samples:
          remaining_ids = random.sample(remaining_ids, k_her_samples)
        # TODO: Incorporate collisions!
        for new_goal_id in remaining_ids:
          # Get a new goal observation
          o_new_goal, _, _, _, _ = local_buffer.get_transition_at(new_goal_id)
          # Extract the goal from the observation
          new_goal = env.env._get_state_from_obs(o_new_goal)
          o_her = env.env._replace_goal_in_obs(o_t, new_goal)
          o2_her = env.env._replace_goal_in_obs(o2_t, new_goal)
          r_her =  env.env._compute_reward(o2_t, 0)
          if r_her == 0:
            d_her = 1
          else:
            d_her = d_t
          replay_buffer.store(o_her, a_t, r_her, o2_her, d_her)
      ## Update handling
      # TODO Make this in parallel?
      if t_total >= update_after: #and (t_total-t_last_update) >= update_every:
        # Update for amount of environment steps done
        for j in range(n_updates):#t_total-t_last_update):
          batch = replay_buffer.sample_batch(batch_size)
          update(data=batch)
        #t_last_update = t_total
    ## End of epoch
    # Save model
    if (epoch % save_freq == 0) or (epoch == n_epochs):
        #logger.save_state({'env': env}, None)
      save_model_training(path=checkpoint_dir, epoch=epoch, ac=ac, ac_targ=ac_targ, replay_buffer=replay_buffer, pi_optimizer=pi_optimizer, q_optimizer=q_optimizer, t_total=t_total)

    # Test the performance of the deterministic version of the agent.
    if perform_test:
      rospy.logwarn("Performing test episode(s).")
      test_agent()

    # Log info about epoch
    if 'Q1Vals' in logger.epoch_dict:
      logger.log_tabular('Epoch', epoch)
      logger.log_tabular('EpRet', with_min_and_max=True)
      logger.log_tabular('GoalReached', average_only=True)
      logger.log_tabular('Collided', average_only=True)
      logger.log_tabular('CriticallyCollided', average_only=True)
      if perform_test:
        logger.log_tabular('TestEpRet', with_min_and_max=True)
      logger.log_tabular('EpLen', average_only=True)
      if perform_test:
        logger.log_tabular('TestEpLen', average_only=True)
      logger.log_tabular('TotalEnvInteracts', t)
      logger.log_tabular('Q1Vals', with_min_and_max=True)
      logger.log_tabular('Q2Vals', with_min_and_max=True)
      logger.log_tabular('LogPi', with_min_and_max=True)
      logger.log_tabular('LossPi', average_only=True)
      logger.log_tabular('LossQ', average_only=True)
      logger.log_tabular('Time', time.time()-start_time)
      #logger.log_tabular('TimeToGoal', average_only=True)
      logger.dump_tabular()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--env', type=str, default='HalfCheetah-v2')
    parser.add_argument('--hid', type=int, default=256)
    parser.add_argument('--l', type=int, default=2)
    parser.add_argument('--gamma', type=float, default=0.99)
    parser.add_argument('--seed', '-s', type=int, default=0)
    parser.add_argument('--epochs', type=int, default=50)
    parser.add_argument('--exp_name', type=str, default='sac')
    args = parser.parse_args()

    from spinup.utils.run_utils import setup_logger_kwargs
    logger_kwargs = setup_logger_kwargs(args.exp_name, args.seed)

    torch.set_num_threads(torch.get_num_threads())

    sac_her(lambda : gym.make(args.env), actor_critic=core.MLPActorCritic,
        ac_kwargs=dict(hidden_sizes=[args.hid]*args.l), 
        gamma=args.gamma, seed=args.seed, n_epochs=args.epochs,
        logger_kwargs=logger_kwargs)
