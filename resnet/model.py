import torch
import numpy as np
from PIL import Image
from torchvision import transforms
import cv2
import time

# Load model
model = torch.hub.load('pytorch/vision:v0.10.0', 'resnet18', pretrained=True)
model.eval()

# Setup preprocessing
preprocess = transforms.Compose([
    transforms.Resize(256),
    transforms.CenterCrop(224),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
])

def process_frame(frame):
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    image = Image.fromarray(frame_rgb)
    
    input_tensor = preprocess(image)
    input_batch = input_tensor.unsqueeze(0)
    
    if torch.cuda.is_available():
        input_batch = input_batch.to('cuda')
        model.to('cuda')
    
    with torch.no_grad():
        output = model(input_batch)
    
    probabilities = torch.nn.functional.softmax(output[0], dim=0)
    top5_prob, top5_catid = torch.topk(probabilities, 5)
    
    return [(categories[idx], prob.item()) for idx, prob in zip(top5_catid, top5_prob)]

# Read the categories
with open("imagenet_classes.txt", "r") as f:
    categories = [s.strip() for s in f.readlines()]

# Initialize video capture
cap = cv2.VideoCapture(0)
last_prediction_time = 0
current_predictions = []

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # Update predictions every 2 seconds
    current_time = time.time_ns()
    if current_time - last_prediction_time >= 500_000_000:
        current_predictions = process_frame(frame)
        last_prediction_time = current_time
    
    # Draw current predictions on frame
    y_position = 30
    for i, (category, prob) in enumerate(current_predictions):
        text = f"{i+1}. {category}: {prob:.3f}"
        cv2.putText(frame, text, (10, y_position), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3)
        cv2.putText(frame, text, (10, y_position), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
        y_position += 30
    
    cv2.imshow('Real-time Classification', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()