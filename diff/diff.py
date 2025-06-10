import cv2
import re
import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter


img_A = cv2.imread('../build/results/cos.png', cv2.IMREAD_UNCHANGED).astype(np.float32)
img_B = cv2.imread('../build/results/nee.png', cv2.IMREAD_UNCHANGED).astype(np.float32)
# img_A = cv2.imread('../build/compare/length_2/cos.png', cv2.IMREAD_UNCHANGED).astype(np.float32)
# img_B = cv2.imread('../build/compare/length_2/bdpt.png', cv2.IMREAD_UNCHANGED).astype(np.float32)

print(f"img_A.shape: {img_A.shape}")
print(f"img_B.shape: {img_B.shape}")

sigma_px = 3

diff = img_A - img_B
diff_gaussian = gaussian_filter(diff, sigma=(sigma_px, sigma_px, 0))
abs_diff_gaussian = np.abs(diff_gaussian)

plt.imshow(diff, vmin=0, vmax=1)
plt.axis('off')
plt.tight_layout()
plt.savefig('./results/diff.png', dpi=200)
plt.show()
plt.close()

plt.imshow(diff_gaussian, vmin=0, vmax=1)
plt.axis('off')
plt.tight_layout()
plt.savefig('./results/diff_gaussian.png', dpi=200)
plt.show()
plt.close()

plt.imshow(abs_diff_gaussian, vmin=0, vmax=1)
plt.axis('off')
plt.tight_layout()
plt.savefig('./results/abs_diff_gaussian.png', dpi=200)
plt.show()
plt.close()

# diff_r = img_A[..., 2] - img_B[..., 2]
# diff_g = img_A[..., 1] - img_B[..., 1]
# diff_b = img_A[..., 0] - img_B[..., 0]

# diff_r_gauss = gaussian_filter(diff_r, sigma=sigma_px)
# diff_g_gauss = gaussian_filter(diff_g, sigma=sigma_px)
# diff_b_gauss = gaussian_filter(diff_b, sigma=sigma_px)

# abs_diff_r_gauss = np.abs(diff_r_gauss)
# abs_diff_g_gauss = np.abs(diff_g_gauss)
# abs_diff_b_gauss = np.abs(diff_b_gauss)

# plt.gray()
# plt.imshow(diff_r,  vmin=0, vmax=1)
# plt.axis('off')
# plt.tight_layout()
# plt.savefig('./results/diff_r.png', dpi=200)
# plt.show()
# plt.close()
# plt.gray()
# plt.imshow(diff_g, vmin=0, vmax=1)
# plt.axis('off')
# plt.tight_layout()
# plt.savefig('./results/diff_g.png', dpi=200)
# plt.show()
# plt.close()
# plt.gray()
# plt.imshow(diff_b,  vmin=0, vmax=1)
# plt.axis('off')
# plt.tight_layout()
# plt.savefig('./results/diff_b.png', dpi=200)
# plt.show()
# plt.close()

# plt.gray()
# plt.imshow(diff_r_gauss,  vmin=0, vmax=1)
# plt.axis('off')
# plt.tight_layout()
# plt.savefig('./results/diff_r_gauss.png', dpi=200)
# plt.show()
# plt.close()
# plt.gray()
# plt.imshow(diff_g_gauss,  vmin=0, vmax=1)
# plt.axis('off')
# plt.tight_layout()
# plt.savefig('./results/diff_g_gauss.png', dpi=200)
# plt.show()
# plt.close()
# plt.gray()
# plt.imshow(diff_b_gauss,  vmin=0, vmax=1)
# plt.axis('off')
# plt.tight_layout()
# plt.savefig('./results/diff_b_gauss.png', dpi=200)
# plt.show()
# plt.close()

# plt.gray()
# plt.imshow(abs_diff_r_gauss,  vmin=0, vmax=1)
# plt.axis('off')
# plt.tight_layout()
# plt.savefig('./results/abs_diff_r_gauss.png', dpi=200)
# plt.show()
# plt.close()
# plt.gray()
# plt.imshow(abs_diff_g_gauss,  vmin=0, vmax=1)
# plt.axis('off')
# plt.tight_layout()
# plt.savefig('./results/abs_diff_g_gauss.png', dpi=200)
# plt.show()
# plt.close()
# plt.gray()
# plt.imshow(abs_diff_b_gauss,  vmin=0, vmax=1)
# plt.axis('off')
# plt.tight_layout()
# plt.savefig('./results/abs_diff_b_gauss.png', dpi=200)
# plt.show()
# plt.close()