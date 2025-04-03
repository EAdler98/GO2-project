import os
import cv2
import numpy as np
from insightface.app import FaceAnalysis
import json
from numpy.linalg import norm
from Person import *
from RS_API import get_XYZ_from_pictures_with_file
import math

def train_faces(dataset_dir, embeddings_file):
    print("training ... ")
    app = FaceAnalysis()
    app.prepare(ctx_id=0, det_size=(640, 640))
    
    embeddings_dict = {}
    
    known_people = os.listdir(dataset_dir)
    
    for person_name in known_people:
        person_dir = os.path.join(dataset_dir, person_name)
        embeddings_dict[person_name] = []
        
        for file_name in os.listdir(person_dir):
            known_image_path = os.path.join(person_dir, file_name)
            known_img = cv2.imread(known_image_path)
            known_faces = app.get(known_img)
            
            for known_face in known_faces:
                known_encoding = known_face.embedding
                embeddings_dict[person_name].append(known_encoding.tolist())
    
    with open(embeddings_file, 'w') as f:
        json.dump(embeddings_dict, f)

def cosine_similarity(a, b):
    return np.dot(a, b) / (norm(a) * norm(b))


def face_orientation_degrees(frame, landmarks):
    size = frame.shape  # (height, width, color_channel)

    image_points = np.array([
        (landmarks[4], landmarks[5]),  # Nose tip
        (landmarks[10], landmarks[11]),  # Chin
        (landmarks[0], landmarks[1]),  # Left eye left corner
        (landmarks[2], landmarks[3]),  # Right eye right corner
        (landmarks[6], landmarks[7]),  # Left Mouth corner
        (landmarks[8], landmarks[9])  # Right mouth corner
    ], dtype="double")

    model_points = np.array([
        (0.0, 0.0, 0.0),  # Nose tip
        (0.0, -330.0, -65.0),  # Chin
        (-165.0, 170.0, -135.0),  # Left eye left corner
        (165.0, 170.0, -135.0),  # Right eye right corner
        (-150.0, -150.0, -125.0),  # Left Mouth corner
        (150.0, -150.0, -125.0)  # Right mouth corner
    ])

    # Camera internals
    center = (size[1] / 2, size[0] / 2)
    focal_length = center[0] / np.tan(60 / 2 * np.pi / 180)
    camera_matrix = np.array(
        [[focal_length, 0, center[0]],
         [0, focal_length, center[1]],
         [0, 0, 1]], dtype="double"
    )

    dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion
    (success, rotation_vector, translation_vector) = cv2.solvePnP(model_points, image_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)

    rvec_matrix = cv2.Rodrigues(rotation_vector)[0]
    proj_matrix = np.hstack((rvec_matrix, translation_vector))
    eulerAngles = cv2.decomposeProjectionMatrix(proj_matrix)[6]

    pitch, yaw, roll = [math.radians(_) for _ in eulerAngles]

    pitch = math.degrees(math.asin(math.sin(pitch)))
    roll = -math.degrees(math.asin(math.sin(roll)))
    yaw = math.degrees(math.asin(math.sin(yaw)))

    return yaw, pitch, roll

def add_new_person_to_dataset(image, face, person_name, dataset_dir):
    """ Add a new person to the dataset by creating a folder with the next number and saving their cropped face image """
    new_person_dir = os.path.join(dataset_dir, person_name)
    os.makedirs(new_person_dir, exist_ok=True)

    # Add a small margin around the face
    margin = 20
    x1, y1, x2, y2 = int(face.bbox[0]), int(face.bbox[1]), int(face.bbox[2]), int(face.bbox[3])
    
    # Crop the face from the image
    cropped_face = image[max(0, y1-margin):min(image.shape[0], y2+margin), max(0, x1-margin):min(image.shape[1], x2+margin)]

    # Save the cropped face in the new directory
    new_image_path = os.path.join(new_person_dir, "1.png")
    cv2.imwrite(new_image_path, cropped_face)
def recognize_faces_from_embeddings(image_path, embeddings_file, dataset_dir):
    app = FaceAnalysis()
    app.prepare(ctx_id=0, det_size=(640, 640))
    
    with open(embeddings_file, 'r') as f:
        embeddings_dict = json.load(f)
    
    img = cv2.imread(image_path)
    faces = app.get(img)
    
    results = []
    
    for face in faces:
        face_name = None
        face_encoding = face.embedding
        
        # Compare the detected face with known embeddings
        for person_name, known_encodings in embeddings_dict.items():
            for known_encoding in known_encodings:
                similarity = cosine_similarity(face_encoding, np.array(known_encoding))
                
                if similarity > 0.6:
                    face_name = person_name
                    break
                    
            if face_name:
                break
        
        if not face_name:
            # Assign a new number for an unknown person
            existing_people = os.listdir(dataset_dir)
            numeric_people = [int(p) for p in existing_people if p.isdigit()]
            next_person_number = max(numeric_people) + 1 if numeric_people else 1
            face_name = str(next_person_number)

            # Add the new person to the dataset, including cropping their face
            add_new_person_to_dataset(img, face, face_name, dataset_dir)

        center_x = int((face.bbox[0] + face.bbox[2]) / 2)
        center_y = int((face.bbox[1] + face.bbox[3]) / 2)
        XYZ = get_XYZ_from_pictures_with_file(center_x, center_y)
        
        # Calculate the Yaw, Pitch, Roll angles
        landmarks = [
            face.kps[0][0], face.kps[0][1],  # left eye
            face.kps[1][0], face.kps[1][1],  # right eye
            face.kps[2][0], face.kps[2][1],  # nose
            face.kps[3][0], face.kps[3][1],  # left mouth corner
            face.kps[4][0], face.kps[4][1],  # right mouth corner
            (face.bbox[2] + face.bbox[0]) / 2, face.bbox[3]  # chin (approximation)
        ]
        yaw, pitch, roll = face_orientation_degrees(img, landmarks)
        
        person = Person(face_name, yaw, [center_x, center_y], XYZ)  # Now includes Yaw
        results.append(person)
    
    return results

def get_persons(num_images):
    dataset_dir = "../DATASET"
    embeddings_file = "face_embeddings.json"
    # images = []
    # for i in range(1, num_images + 1):
    image = "aligned_rgb_image_"+str(1)+".png"
    # images.append(image)
    
    persons_dict = {}
    # for image in images:
    print("start recognize_faces")
    persons : list[Person] = recognize_faces_from_embeddings(image, embeddings_file, dataset_dir)

    for f in persons:
        if f.name not in persons_dict:
            persons_dict[f.name] = f
                
    persons_array = list(persons_dict.values())  
    return persons_array
