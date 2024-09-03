using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Training_Area_Initialize : MonoBehaviour
{
    [SerializeField] GameObject TrainingAreaPrefab;  // The prefab for the training area

    private void Start()
    {
        InitializeTrainingAreas(Drone_Values.TrainingAreaNumber, Drone_Values.TrainingAreaSize);
    }

    private void InitializeTrainingAreas(int numberOfAreas, float areaSize)
    {
        int right = 0;
        int left = 1;
        for (int i = 0; i < numberOfAreas; i++)
        {
            Vector3 position = Vector3.zero;
            if (i % 2 == 0)
            {
                // Calculate position for each training area
                position = new Vector3(-right * areaSize * 2 * 2f, 0, 0); // Arranging areas in a line along the x-axis
                right++;
            }
            else
            {
                position = new Vector3(left * areaSize * 2 * 2f, 0, 0); // Arranging areas in a line along the x-axis
                left++;
            }
            

            // Instantiate the training area at the calculated position
            GameObject trainingArea = Instantiate(TrainingAreaPrefab, position, Quaternion.identity);

            // Set the instantiated training area's parent to be this MainGameObject
            trainingArea.transform.SetParent(this.transform);

            // Optionally, name the training areas for easy identification
            trainingArea.name = "TrainingArea_" + i;
        }
    }
}
