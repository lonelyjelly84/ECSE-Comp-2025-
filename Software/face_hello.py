import cv2
import numpy as np
import os
import pickle
from pathlib import Path

class OpenCVFaceRecognitionSystem:
  def __init__(self, known_faces_dir="known_faces", encodings_file="opencv_face_data.pkl"):
    self.known_faces_dir = known_faces_dir
    self.encodings_file = encodings_file
    self.known_face_encodings = []
    self.known_face_names = []
    
    # Initialize face detection and recognition
    self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    self.face_recognizer = cv2.face.LBPHFaceRecognizer_create()
    
    # Create known faces directory if it doesn't exist
    Path(known_faces_dir).mkdir(exist_ok=True)
    
    # Load or create face data
    self.load_face_data()
  
  def load_face_data(self):
    """Load face data from file or create new data"""
    if os.path.exists(self.encodings_file):
      print("Loading existing face data...")
      try:
        with open(self.encodings_file, 'rb') as f:
          data = pickle.load(f)
          self.known_face_names = data['names']
        
        # Load the trained recognizer
        self.face_recognizer.read(self.encodings_file.replace('.pkl', '_model.yml'))
        print(f"Loaded face data for: {', '.join(self.known_face_names)}")
      except:
        print("Error loading existing data, creating new...")
        self.create_face_data()
    else:
      print("Creating new face data...")
      self.create_face_data()

  def create_face_data(self):
    """Create face data from images in known_faces directory"""
    faces = []
    labels = []
    self.known_face_names = []
    
    # Supported image extensions
    extensions = ['.jpg', '.jpeg', '.png', '.bmp']
    
    label_id = 0
      
    for filename in os.listdir(self.known_faces_dir):
      if any(filename.lower().endswith(ext) for ext in extensions):
        # Extract name from filename (without extension)
        name = os.path.splitext(filename)[0]
        
        # Load image
        image_path = os.path.join(self.known_faces_dir, filename)
        image = cv2.imread(image_path)
        
        if image is None:
          print(f"Warning: Could not load {filename}")
          continue
        
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Detect faces in the image
        detected_faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)
        
        if len(detected_faces) > 0:
          # Use the largest face found
          largest_face = max(detected_faces, key=lambda x: x[2] * x[3])
          x, y, w, h = largest_face
          
          # Extract face region
          face_roi = gray[y:y+h, x:x+w]
          
          # Resize to standard size
          face_roi = cv2.resize(face_roi, (200, 200))
          
          faces.append(face_roi)
          labels.append(label_id)
          self.known_face_names.append(name)
          
          print(f"Added face data for: {name}")
          label_id += 1
      else:
        print(f"Warning: No face found in {filename}")
    
    if faces:
      # Train the face recognizer
      self.face_recognizer.train(faces, np.array(labels))
      self.save_face_data()
    else:
      print("No faces found in the known_faces directory!")

  def save_face_data(self):
    """Save face data to files"""
    # Save names mapping
    data = {'names': self.known_face_names}
    with open(self.encodings_file, 'wb') as f:
      pickle.dump(data, f)
    
    # Save trained model
    model_file = self.encodings_file.replace('.pkl', '_model.yml')
    self.face_recognizer.save(model_file)
    
    print(f"Saved face data for {len(self.known_face_names)} people")
  
  def recognize_faces(self, frame):
    """Recognize faces in a frame"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect faces
    faces = self.face_cascade.detectMultiScale(gray, 1.1, 4, minSize=(50, 50))
    
    face_locations = []
    face_names = []
    
    for (x, y, w, h) in faces:
      # Extract face region
      face_roi = gray[y:y+h, x:x+w]
      face_roi = cv2.resize(face_roi, (200, 200))
      
      # Predict identity
      if len(self.known_face_names) > 0:
        label, confidence = self.face_recognizer.predict(face_roi)
        
        # Lower confidence values mean better matches
        # Typical range: 0-100 (lower is better)
        if confidence < 70:  # Adjustable threshold
          name = self.known_face_names[label]
          confidence_percent = max(0, 100 - confidence)
          name = f"{name} ({confidence_percent:.0f}%)"
        else:
          name = "Unknown"
      else:
        name = "Unknown"
      
      # Convert to format compatible with drawing function
      face_locations.append((y, x + w, y + h, x))  # top, right, bottom, left
      face_names.append(name)
    
    return face_locations, face_names
  
  def draw_face_boxes(self, frame, face_locations, face_names):
    """Draw bounding boxes and names on faces"""
    for (top, right, bottom, left), name in zip(face_locations, face_names):
      # Draw rectangle around face
      color = (0, 255, 0) if "Unknown" not in name else (0, 0, 255)
      cv2.rectangle(frame, (left, top), (right, bottom), color, 2)
      
      # Draw label with name
      cv2.rectangle(frame, (left, bottom - 35), (right, bottom), color, cv2.FILLED)
      font = cv2.FONT_HERSHEY_DUPLEX
      cv2.putText(frame, name, (left + 6, bottom - 6), font, 0.6, (255, 255, 255), 1)
    
    return frame
  
  def add_new_face(self, name, image_path):
    """Add a new face to the known faces database"""
    if os.path.exists(image_path):
      # Copy image to known_faces directory
      import shutil
      new_path = os.path.join(self.known_faces_dir, f"{name}.jpg")
      shutil.copy2(image_path, new_path)
      
      # Recreate face data
      self.create_face_data()
      print(f"Added new face: {name}")
      return True
    else:
      print(f"Image not found: {image_path}")
      return False
  
  def run_live_recognition(self):
    """Main function to run live face recognition"""
    # Initialize video capture
    video_capture = cv2.VideoCapture(0)
    
    if not video_capture.isOpened():
      print("Error: Could not open video capture")
      return
    
    print("Starting live face recognition... Press 'q' to quit, 'r' to retrain")
    print(f"Known faces: {', '.join(self.known_face_names) if self.known_face_names else 'None'}")
    
    while True:
      # Capture frame
      ret, frame = video_capture.read()
      if not ret:
        print("Error: Could not read frame")
        break
      
      # Recognize faces
      face_locations, face_names = self.recognize_faces(frame)
      
      # Draw face boxes and names
      frame = self.draw_face_boxes(frame, face_locations, face_names)
      
      # Add instructions on screen
      cv2.putText(frame, "Press 'q' to quit, 'r' to retrain", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
      
      # Display frame
      cv2.imshow('OpenCV Face Recognition', frame)
      
      # Handle key presses
      key = cv2.waitKey(1) & 0xFF
      if key == ord('q'):
        break
      elif key == ord('r'):
        print("Retraining...")
        self.create_face_data()
        print("Retraining complete!")
    
    # Clean up
    video_capture.release()
    cv2.destroyAllWindows()
    print("Face recognition stopped")
  def recognize_single_face(self, min_confidence=45, timeout_seconds=30):
    """
    Recognize a single face and return the name when confidence >= min_confidence
    
    Args:
      min_confidence: Minimum confidence percentage required for a match (default: 45%)
      timeout_seconds: Maximum time to wait for a match (default: 30 seconds)
    
    Returns:
      str: Name of recognized person, or None if no match found within timeout
    """
    # Initialize video capture
    video_capture = cv2.VideoCapture(0)
    
    if not video_capture.isOpened():
      print("Error: Could not open video capture")
      return None
    
    print(f"Starting face recognition... Looking for {min_confidence}% confidence match")
    print(f"Known faces: {', '.join(self.known_face_names) if self.known_face_names else 'None'}")
    print("Press 'q' to quit early")
    
    import time
    start_time = time.time()
    
    while True:
      # Check timeout
      if time.time() - start_time > timeout_seconds:
        print(f"Timeout after {timeout_seconds} seconds")
        break
      
      # Capture frame
      ret, frame = video_capture.read()
      if not ret:
        print("Error: Could not read frame")
        break
      
      # Recognize faces
      face_locations, face_names = self.recognize_faces(frame)
      
      # Check for matches with sufficient confidence
      for name in face_names:
        if "Unknown" not in name and "%" in name:
          # Extract confidence percentage from name
          confidence_str = name.split("(")[1].split("%")[0]
          try:
            confidence = float(confidence_str)
            if confidence >= min_confidence:
              # Extract just the name without confidence
              person_name = name.split(" (")[0]
              print(f"Match found: {person_name} with {confidence:.0f}% confidence")
              
              # Clean up
              video_capture.release()
              cv2.destroyAllWindows()
              return person_name
          except ValueError:
            continue
      
      # Draw face boxes and names for visual feedback
      frame = self.draw_face_boxes(frame, face_locations, face_names)
      
      # Add status text
      elapsed_time = int(time.time() - start_time)
      remaining_time = timeout_seconds - elapsed_time
      status_text = f"Looking for {min_confidence}% match... {remaining_time}s remaining"
      cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
      cv2.putText(frame, "Press 'q' to quit", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
      
      # Display frame
      cv2.imshow('Face Recognition - Waiting for Match', frame)
      
      # Handle key presses
      key = cv2.waitKey(1) & 0xFF
      if key == ord('q'):
        print("Recognition cancelled by user")
        break
    
    # Clean up
    video_capture.release()
    cv2.destroyAllWindows()
    print("No match found")
    return None

def main():
  """Example usage"""
  # Create face recognition system
  face_system = OpenCVFaceRecognitionSystem()
  
  # Check if we have any known faces
  if not face_system.known_face_names:
    print("\n" + "="*50)
    print("SETUP REQUIRED:")
    print("1. Create a 'known_faces' folder in the same directory")
    print("2. Add photos of people you want to recognize")
    print("3. Name the files with the person's name (e.g., 'john.jpg', 'sarah.png')")
    print("4. Run the script again")
    print("="*50)
    
    # Create the directory for user convenience
    os.makedirs("known_faces", exist_ok=True)
    return
  
  # Start live recognition
  face_system.run_live_recognition()

if __name__ == "__main__":
  # Create the face recognition system
  face_system = OpenCVFaceRecognitionSystem()
  
  # Check if we have known faces
  if not face_system.known_face_names:
    print("No known faces found. Please add photos to the 'known_faces' folder.")
  else:
    # Use the recognize_single_face method
    name = face_system.recognize_single_face(min_confidence=45, timeout_seconds=30)
    
    if name:
      print(f"Hello, {name}!")
    else:
      print("No one recognized")