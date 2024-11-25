import base64
import cv2
import openai

def identify_image_with_openai(image):
    # Convert numpy array to jpg format
    _, buffer = cv2.imencode('.jpg', image)
    # Convert to base64
    base64_image = base64.b64encode(buffer).decode('utf-8')
    
    # Carefully crafted prompt for accurate material classification
    prompt = """You are a precise recycling material classifier. Analyze this image and categorize it into exactly ONE of these categories:
    0: Glass (transparent containers, bottles)
    1: Metal (cans, bottles, aluminum, steel)
    2: Paper (cardboard, newspapers, magazines)
    3: Plastic (bottles, containers, packaging)
    4: Waste (non-recyclable items)

    Important instructions:
    - Consider texture, transparency, reflectiveness, and shape
    - Look for material-specific characteristics
    - Respond ONLY with the number (0-4) corresponding to the category
    - If uncertain, classify as waste, however try to avoid this case if possible.
    
    Classify this item:"""

    try:
        client = openai.OpenAI(api_key='')
        response = client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        {
                            "type": "image_url",
                            "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"},
                        },
                    ],
                }
            ],
        )
        
        # Extract the number from the response
        result = response.choices[0].message.content.strip()
        return int(result)
    
    except Exception as e:
        print(f"Error in image classification: {str(e)}")
        return -1  # Return -1 to indicate error
    
if __name__ == "__main__":
    import cv2
    image = cv2.imread("image.png")
    print(identify_image_with_openai(image))