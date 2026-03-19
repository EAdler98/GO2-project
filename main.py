
from typing import List
from Person import Person
import numpy as np
from recognize_persons import get_persons, train_faces

def deciding(persons: List[Person]):
    if len(persons) ==0:
        return -1
    person = persons[0]
    return person

if __name__ =="__main__":
    
    

    print("start!")
    # train_model("DATASET")
    NO_PICUTURE=4
    #take images and proccess them, create median depth image
    # create_images(NO_PICUTURE)
    
    #process images captured
    train_faces("DATASET", "face_embeddings.json")
    persons = get_persons(NO_PICUTURE)
    #train_faces("DATASET", "face_embeddings.json")
    
    # print("FINISH")
    # print(len(persons))
    person = deciding(persons)
    
    print("the person: " , person.name)
    # person = Person(name="EVYATAR", X_angle=10.955772245772206, center_XY=[871, 96], XYZ=(92.558642578125, 377.70000000000005, 149.5942941345215) )
    train_faces("DATASET", "face_embeddings.json")
    print("deciding(persons): ", person)
    print("start go!!")
    print(person.name,person.X_angle_degrees,person.real_XYZ[0],person.real_XYZ[1],person.real_XYZ[2])
    
    # init_sport()
    
    # first_angle, distance,second_angle=calculate_route(person)
    # print(first_angle,distance,second_angle)
    
    