// -*- lsst-c++ -*/
/***
 * @file skeleton_node.h
 * @brief Defines the skeleton node class
 * @version 0.1
 * @copyright MIT License. Please see package.xml for further detail.
 */

#include <string>
#include <vector>

#ifndef SKELETON_NODE_H
#define SKELETON_NODE_H
 
namespace obstacles {

/**
 * @brief The Kalman filter super class
 */
class SkeletonNode {
 private:
  /**
   * @brief The name and identifier of this bone.
   */
  std::string name;

  /**
   * @brief The children of this bone
   */
  std::vector<SkeletonNode*> children;

 public: 
  /**
   * @brief Default constructor
   */
  SkeletonNode(){};

  /**
   * @brief Default constructor
   * 
   * @param[in] name The bone name (identifier)
   */ 
  SkeletonNode(std::string name):
      name(name)
      {}

  /**
   * @brief Destructor.
   */
  virtual ~SkeletonNode(){}

  /**
   * @brief Return the name of the bone.
   */
  inline std::string Name() { return name; }
  
  /**
   * @brief Return all children.
   */
  inline std::vector<SkeletonNode*> GetChildren() { return children; }

  /**
   * @brief Return the n-th child.
   */
  inline SkeletonNode* GetChildAt(int n) { return children.at(n); }

  /**
   * @brief Add a child to the children
   */
  inline void AddChild(SkeletonNode* child) { children.push_back(child); }
};
} // namespace obstacles
#endif // SKELETON_NODE_H