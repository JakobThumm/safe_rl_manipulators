import numpy as np
import pandas as pd
import copy
import argparse
import os


def path_check(string):
    if os.path.isfile(string):
        return string
    else:
        raise FileNotFoundError(string)


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='Smooth results with sliding window approach.')
  parser.add_argument('path', type=path_check, help='Path to file')
  parser.add_argument('--window_size', type=int, default=5, help='Size of sliding window')

  args = parser.parse_args()
  window_size = args.window_size
  filepath = args.path

  assert window_size > 1
  if window_size % 2 == 0:
    window_size -= 1
  start = int(np.floor(window_size/2))

  data = pd.read_table(filepath, delimiter='\t')
  old_data = data.copy()
  length = data.shape[0]
  end = length-int(np.ceil(window_size/2))
  for col in data.columns:
    if col != 'Epoch':
      values = old_data[col].values
      for i in range(start, end):
        data[col][i] = np.mean(values[i-start:i+start+1])

  dirname = os.path.dirname(filepath)
  filename, extension = os.path.splitext(os.path.basename(filepath))
  out_file = dirname + "/" + filename + "_smoothed" + extension
  data.to_csv(out_file, index=False, sep='\t') 
