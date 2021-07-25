// A pause menu for the simulator to give the user more control over the scene and easy access to the Main menu.

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class PauseGame : MonoBehaviour
{
    public bool gamePaused = false;
    public GameObject pauseMenu;
    public AudioSource pauseSound;

    // Update is called once per frame
    void Update()
    {
        if (Input.GetButtonDown("Cancel")) // When escape is pressed, toggles the pause menu
        {
            if (gamePaused == false)
            {
                Time.timeScale = 0; // i.e. stos the time in simulator, thus pausing the game.
                gamePaused = true;
                Cursor.visible = true;
                pauseMenu.SetActive(true);
                pauseSound.Play();
            }
            else
            {
                Cursor.visible = false;
                gamePaused = false;
                Time.timeScale = 1;
                pauseMenu.SetActive(false);
            }
        }
    }
    public void resumeGame() // Closes the pause menu and resumes the simulator
    {
        Cursor.visible = false;
        gamePaused = false;
        Time.timeScale = 1;
        pauseMenu.SetActive(false);
    }

    public void restartGame() // Restart the simulator and resets all the parameters. 
    {
        Cursor.visible = false;
        gamePaused = false;
        Time.timeScale = 1;
        pauseMenu.SetActive(false);
        SceneManager.LoadScene(1);
    }

    public void quitGame() // Quit Game
    {
        Cursor.visible = false;
        gamePaused = false;
        Time.timeScale = 1;
        pauseMenu.SetActive(false);
        SceneManager.LoadScene(0);
    }
}
