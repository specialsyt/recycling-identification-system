import torch
import torch
from torchvision import models, transforms
from PIL import Image

# Load the pre-trained model and weights
model = models.mobilenet_v3_small(pretrained=True)
model.eval()



# Given an image, classify it into one of the following categories:
# 1. Metal
# 2. Plastic
# 3. Glass
# 4. Paper
# 5. Cardboard
# 6. Other

