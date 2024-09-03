using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

public class Training_Area_Initialize : MonoBehaviour
{
    [SerializeField] GameObject TrainingAreaPrefab;  // The prefab for the training area

    private void Start()
    {
        InitializeTrainingAreas(Drone_Values.TrainingAreaNumber, Drone_Values.TrainingAreaSize);
    }

    private void InitializeTrainingAreas(int numberOfAreas, float areaSize)
    {



        if (numberOfAreas > 0)
        {
            int leftRight = (numberOfAreas - 1) / 2;
            int upDown = (numberOfAreas - 1) - leftRight;
            int right = 1;
            int left = 1;
            int up = 1;
            int down = 1;

            Vector3 position = Vector3.zero;
            // Instantiate the training area at the calculated position
            GameObject trainingArea = Instantiate(TrainingAreaPrefab, position, Quaternion.identity);

            // Set the instantiated training area's parent to be this MainGameObject
            trainingArea.transform.SetParent(this.transform);

            // Optionally, name the training areas for easy identification
            trainingArea.name = "TrainingArea_" + 0;


            for (int i = 0; i < leftRight; i++)
            {
                position = Vector3.zero;

                if (i % 2 == 0)
                {
                    position = new Vector3(-right * areaSize * 2 * 2f, 0, 0); // Arranging areas in a line along the x-axis
                    right++;
                }
                else
                {
                    position = new Vector3(left * areaSize * 2 * 2f, 0, 0); // Arranging areas in a line along the x-axis
                    left++;
                }
                // Instantiate the training area at the calculated position
                trainingArea = Instantiate(TrainingAreaPrefab, position, Quaternion.identity);

                // Set the instantiated training area's parent to be this MainGameObject
                trainingArea.transform.SetParent(this.transform);

                // Optionally, name the training areas for easy identification
                trainingArea.name = $"TrainingArea_right{right - 1}_left{left - 1}";
            }

            for (int i = 0; i < upDown; i++)
            {
                position = Vector3.zero;

                if (i % 2 == 0)
                {
                    position = new Vector3(0, 0, up * areaSize * 2 * 2f); // Arranging areas in a line along the x-axis
                    up++;
                }
                else
                {
                    position = new Vector3(0, 0, -down * areaSize * 2 * 2f); // Arranging areas in a line along the x-axis
                    down++;
                }
                // Instantiate the training area at the calculated position
                trainingArea = Instantiate(TrainingAreaPrefab, position, Quaternion.identity);

                // Set the instantiated training area's parent to be this MainGameObject
                trainingArea.transform.SetParent(this.transform);

                // Optionally, name the training areas for easy identification
                trainingArea.name = $"TrainingArea_up{up - 1}_down{down - 1}";
            }
        }
        


        //// Calculate the midpoint for splitting the areas
        //int halfNumberOfAreas = numberOfAreas / 2;

        //for (int i = 0; i < numberOfAreas; i++)
        //{
        //    Vector3 position = Vector3.zero;
        //    if (i < halfNumberOfAreas)
        //    {
        //        // Arrange the first half of the areas in a left-right line
        //        if (i % 2 == 0)
        //        {
        //            position = new Vector3(-right * areaSize * 2 * 2f, 0, 0); // Arranging areas in a line along the x-axis
        //            right++;
        //        }
        //        else
        //        {
        //            position = new Vector3(left * areaSize * 2 * 2f, 0, 0); // Arranging areas in a line along the x-axis
        //            left++;
        //        }
        //    }
        //    else
        //    {
        //        // Arrange the second half of the areas in an up-down line
        //        if ((i - halfNumberOfAreas) % 2 == 0)
        //        {
        //            position = new Vector3(0, 0, up * areaSize * 2 * 2f); // Arranging areas in a line along the y-axis
        //            up++;
        //        }
        //        else
        //        {
        //            position = new Vector3(0, 0, -down * areaSize * 2 * 2f); // Arranging areas in a line along the y-axis
        //            down++;
        //        }
        //    }

        //    // Instantiate the training area at the calculated position
        //    GameObject trainingArea = Instantiate(TrainingAreaPrefab, position, Quaternion.identity);

        //    // Set the instantiated training area's parent to be this MainGameObject
        //    trainingArea.transform.SetParent(this.transform);

        //    // Optionally, name the training areas for easy identification
        //    trainingArea.name = "TrainingArea_" + i;
        //}
    }
}
