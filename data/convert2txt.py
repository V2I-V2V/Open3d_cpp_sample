import numpy as np

npz = np.load("000001.npz")
pos0 = npz["pos1"]
pos1 = npz["pos2"]
print(pos0.shape, pos1.shape)

np.savetxt("pos0.txt", pos0)
np.savetxt("pos1.txt", pos1)

prevf = np.load("prev_frame.npy")[:, :3]
nextf = np.load("next_frame.npy")[:, :3]

np.savetxt("prevf.txt", prevf)
np.savetxt("nextf.txt", nextf)

