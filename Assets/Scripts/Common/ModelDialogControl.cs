using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class ModelDialogControl : MonoBehaviour
{
    public GameObject dialogBox;
    public Button OKButton;
    public TMP_Text TitleLable;
    public TMP_Text ContentLable;

    // Start is called before the first frame update
    void Start()
    {
        dialogBox.SetActive(false);
        OKButton.onClick.AddListener(HideDialog);
    }

    public void ShowDialog(string title, string content)
    {
        dialogBox.SetActive(true); // 显示对话框
        TitleLable.text = title;
        ContentLable.text = content;
    }

    public void HideDialog()
    {
        dialogBox.SetActive(false); // 隐藏对话框
    }
}
