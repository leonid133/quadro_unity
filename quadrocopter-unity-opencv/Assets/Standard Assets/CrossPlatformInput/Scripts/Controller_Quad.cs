using UnityEngine;
using System.Collections;

public class Controller_Quad : MonoBehaviour {

    CharacterController contr;
    private Vector3 deltaDirection;

    // Use this for initialization
	void Start () {
	
	}

    void LateUpdate()
    {
        /*
        transform.eulerAngles.y += Input.GetAxis("Horizontal") * 3;  //входящие инпуты
        contr = GetComponent(CharacterController);        //взаимодействие со стандатрным чар контр
        deltaDirection = Vector3(0, 0, Input.GetAxis("Vertical")) * 0.15; //входящие инпуты
        contr.Move(deltaDirection * Time.deltaTime);   //взаимодействие со стандатрным чар контр - движется потому что в функции апдейт
        */
    }

    // Update is called once per frame
    void Update () {
	
	}
}
