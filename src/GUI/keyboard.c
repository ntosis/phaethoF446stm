
#include <guiLibs/DIALOG.h>
#include "spi.h"
#include "string.h"
extern WM_HWIN hWin;
extern int id_edit_0;
/*********************************************************************
*
*       Defines
*
**********************************************************************
*/
#define GUI_MAKE_COLOR(ABGR) (((((U32)ABGR) & 0xFF000000ul) ^ 0xFF000000ul) | ((((U32)ABGR) & 0x00FF0000ul) >> 16) | (((U32)ABGR) & 0x0000FF00ul) | ((((U32)ABGR) & 0x000000FFul) << 16))


#define ID_WINDOW_0            (GUI_ID_USER + 0x00)
#define ID_BUTTON_0            (GUI_ID_USER + 0x01)  // a
#define ID_BUTTON_1            (GUI_ID_USER + 0x02)  // b
#define ID_BUTTON_2            (GUI_ID_USER + 0x03)  // c
#define ID_BUTTON_3            (GUI_ID_USER + 0x04)  // etc..
#define ID_BUTTON_4            (GUI_ID_USER + 0x05)
#define ID_BUTTON_5            (GUI_ID_USER + 0x06)
#define ID_BUTTON_6            (GUI_ID_USER + 0x07)
#define ID_BUTTON_7            (GUI_ID_USER + 0x08)
#define ID_BUTTON_8            (GUI_ID_USER + 0x09)
#define ID_BUTTON_9            (GUI_ID_USER + 0x0A)
#define ID_BUTTON_10           (GUI_ID_USER + 0x0B)
#define ID_BUTTON_11           (GUI_ID_USER + 0x0C)
#define ID_BUTTON_12           (GUI_ID_USER + 0x0D)
#define ID_BUTTON_13           (GUI_ID_USER + 0x0E)
#define ID_BUTTON_14           (GUI_ID_USER + 0x0F)
#define ID_BUTTON_15           (GUI_ID_USER + 0x10)
#define ID_BUTTON_16           (GUI_ID_USER + 0x11)
#define ID_BUTTON_17           (GUI_ID_USER + 0x12)
#define ID_BUTTON_18           (GUI_ID_USER + 0x13)
#define ID_BUTTON_19           (GUI_ID_USER + 0x14)
#define ID_BUTTON_20           (GUI_ID_USER + 0x15)
#define ID_BUTTON_21           (GUI_ID_USER + 0x16)
#define ID_BUTTON_22           (GUI_ID_USER + 0x17)
#define ID_BUTTON_23           (GUI_ID_USER + 0x18)
#define ID_BUTTON_24           (GUI_ID_USER + 0x19)
#define ID_BUTTON_25           (GUI_ID_USER + 0x1A)  // z
#define ID_BUTTON_26           (GUI_ID_USER + 0x1B)  // Back
#define ID_BUTTON_27           (GUI_ID_USER + 0x1C)  // Clr
#define ID_BUTTON_28           (GUI_ID_USER + 0x1D)  // Space

#define ID_WINDOW_1            (GUI_ID_USER + 0x20)
#define ID_EDIT_0              (GUI_ID_USER + 0x88)

#define BUTTON_X_SIZE           30
#define BUTTON_Y_SIZE           30

#define BUTTON_SPACING          5

#define BUTTON_X_POS(x)         BUTTON_SPACING + (x * (BUTTON_X_SIZE + BUTTON_SPACING))

#define OFFSET_SECOND_ROW_X     BUTTON_X_SIZE / 2 + BUTTON_SPACING
#define OFFSET_SECOND_ROW_Y     BUTTON_Y_SIZE + (BUTTON_SPACING * 2)

#define OFFSET_THIRD_ROW_X      BUTTON_X_SIZE + BUTTON_SPACING * 2
#define OFFSET_THIRD_ROW_Y     (BUTTON_Y_SIZE * 2) + (BUTTON_SPACING * 3)

#define COLOR_BUTTON_FRAME_0    GUI_MAKE_COLOR(0x1AC39D)
#define COLOR_BUTTON_FRAME_1    GUI_MAKE_COLOR(0x275BEA)

#define COLOR_BUTTON_BK_0       GUI_MAKE_COLOR(0x613600)
#define COLOR_BUTTON_BK_1       GUI_MAKE_COLOR(0x613600)




/*********************************************************************
*
*       Static data
*
**********************************************************************
*/
typedef struct {
  GUI_COLOR aColorFrame[2];
  GUI_COLOR aColorBK[2];
} COLORS;

static COLORS _Colors = {
  { COLOR_BUTTON_FRAME_0, COLOR_BUTTON_FRAME_1},
  { COLOR_BUTTON_BK_0,    COLOR_BUTTON_BK_1   }
};

/*********************************************************************
*
*       _aKeyboardDialogCreate
*/
static const GUI_WIDGET_CREATE_INFO _aKeyboardDialogCreate[] = {
  { WINDOW_CreateIndirect, "Window", ID_WINDOW_0, 0, 0, (BUTTON_X_SIZE + BUTTON_SPACING) * 10 + BUTTON_SPACING, (BUTTON_Y_SIZE + BUTTON_SPACING) * 3 + BUTTON_SPACING, 0, 0x0, 0 },
  //
  // First row
  //
  { BUTTON_CreateIndirect, "q", ID_BUTTON_16, BUTTON_X_POS(0), BUTTON_SPACING, BUTTON_X_SIZE, BUTTON_Y_SIZE, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "w", ID_BUTTON_22, BUTTON_X_POS(1), BUTTON_SPACING, BUTTON_X_SIZE, BUTTON_Y_SIZE, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "e", ID_BUTTON_4,  BUTTON_X_POS(2), BUTTON_SPACING, BUTTON_X_SIZE, BUTTON_Y_SIZE, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "r", ID_BUTTON_17, BUTTON_X_POS(3), BUTTON_SPACING, BUTTON_X_SIZE, BUTTON_Y_SIZE, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "t", ID_BUTTON_19, BUTTON_X_POS(4), BUTTON_SPACING, BUTTON_X_SIZE, BUTTON_Y_SIZE, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "y", ID_BUTTON_24, BUTTON_X_POS(5), BUTTON_SPACING, BUTTON_X_SIZE, BUTTON_Y_SIZE, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "u", ID_BUTTON_20, BUTTON_X_POS(6), BUTTON_SPACING, BUTTON_X_SIZE, BUTTON_Y_SIZE, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "i", ID_BUTTON_8,  BUTTON_X_POS(7), BUTTON_SPACING, BUTTON_X_SIZE, BUTTON_Y_SIZE, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "o", ID_BUTTON_14, BUTTON_X_POS(8), BUTTON_SPACING, BUTTON_X_SIZE, BUTTON_Y_SIZE, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "p", ID_BUTTON_15, BUTTON_X_POS(9), BUTTON_SPACING, BUTTON_X_SIZE, BUTTON_Y_SIZE, 0, 0x0, 0 },
  //
  // Second row
  //
  { BUTTON_CreateIndirect, "a", ID_BUTTON_0,  OFFSET_SECOND_ROW_X + BUTTON_X_POS(0), OFFSET_SECOND_ROW_Y, BUTTON_X_SIZE, BUTTON_Y_SIZE, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "s", ID_BUTTON_18, OFFSET_SECOND_ROW_X + BUTTON_X_POS(1), OFFSET_SECOND_ROW_Y, BUTTON_X_SIZE, BUTTON_Y_SIZE, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "d", ID_BUTTON_3,  OFFSET_SECOND_ROW_X + BUTTON_X_POS(2), OFFSET_SECOND_ROW_Y, BUTTON_X_SIZE, BUTTON_Y_SIZE, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "f", ID_BUTTON_5,  OFFSET_SECOND_ROW_X + BUTTON_X_POS(3), OFFSET_SECOND_ROW_Y, BUTTON_X_SIZE, BUTTON_Y_SIZE, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "g", ID_BUTTON_6,  OFFSET_SECOND_ROW_X + BUTTON_X_POS(4), OFFSET_SECOND_ROW_Y, BUTTON_X_SIZE, BUTTON_Y_SIZE, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "h", ID_BUTTON_7,  OFFSET_SECOND_ROW_X + BUTTON_X_POS(5), OFFSET_SECOND_ROW_Y, BUTTON_X_SIZE, BUTTON_Y_SIZE, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "j", ID_BUTTON_9,  OFFSET_SECOND_ROW_X + BUTTON_X_POS(6), OFFSET_SECOND_ROW_Y, BUTTON_X_SIZE, BUTTON_Y_SIZE, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "k", ID_BUTTON_10, OFFSET_SECOND_ROW_X + BUTTON_X_POS(7), OFFSET_SECOND_ROW_Y, BUTTON_X_SIZE, BUTTON_Y_SIZE, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "l", ID_BUTTON_11, OFFSET_SECOND_ROW_X + BUTTON_X_POS(8), OFFSET_SECOND_ROW_Y, BUTTON_X_SIZE, BUTTON_Y_SIZE, 0, 0x0, 0 },
  //
  // Third row
  //
  { BUTTON_CreateIndirect, "\' \'", ID_BUTTON_28, BUTTON_X_POS(0), OFFSET_THIRD_ROW_Y, BUTTON_X_SIZE, BUTTON_Y_SIZE, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "z",     ID_BUTTON_25, BUTTON_X_POS(1), OFFSET_THIRD_ROW_Y, BUTTON_X_SIZE, BUTTON_Y_SIZE, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "x",     ID_BUTTON_23, BUTTON_X_POS(2), OFFSET_THIRD_ROW_Y, BUTTON_X_SIZE, BUTTON_Y_SIZE, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "c",     ID_BUTTON_2,  BUTTON_X_POS(3), OFFSET_THIRD_ROW_Y, BUTTON_X_SIZE, BUTTON_Y_SIZE, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "v",     ID_BUTTON_21, BUTTON_X_POS(4), OFFSET_THIRD_ROW_Y, BUTTON_X_SIZE, BUTTON_Y_SIZE, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "b",     ID_BUTTON_1,  BUTTON_X_POS(5), OFFSET_THIRD_ROW_Y, BUTTON_X_SIZE, BUTTON_Y_SIZE, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "n",     ID_BUTTON_13, BUTTON_X_POS(6), OFFSET_THIRD_ROW_Y, BUTTON_X_SIZE, BUTTON_Y_SIZE, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "m",     ID_BUTTON_12, BUTTON_X_POS(7), OFFSET_THIRD_ROW_Y, BUTTON_X_SIZE, BUTTON_Y_SIZE, 0, 0x0, 0 },
  //
  // Some special commands
  //
  { BUTTON_CreateIndirect, "<-",    ID_BUTTON_26, BUTTON_X_POS(8), OFFSET_THIRD_ROW_Y, BUTTON_X_SIZE, BUTTON_Y_SIZE, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Clr",   ID_BUTTON_27, BUTTON_X_POS(9), OFFSET_THIRD_ROW_Y, BUTTON_X_SIZE, BUTTON_Y_SIZE, 0, 0x0, 0 },
};

/*********************************************************************
*
*       _aDialogCreate
*/
static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] = {
  { WINDOW_CreateIndirect, "Window", ID_WINDOW_1,  0,  0, 480, 272, 0, 0x0, 0 },
  { EDIT_CreateIndirect,   "Edit",   ID_EDIT_0,   10, 10, 460,  20, 0, 0x64, 0 },
};

/*********************************************************************
*
*       _hKeyboard
*/
WM_HWIN _hKeyboard;  // If it is know module wide itis easier to handle, in this case

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/
 /*********************************************************************
*
*       _cbButton
*/
static void _cbButton(WM_MESSAGE * pMsg) {
  GUI_RECT Rect;
  int      State;
  char     acText[4];

  switch (pMsg->MsgId) {
  case WM_PAINT:
    //
    // Handle the look of the buttons
    //
    WM_GetClientRect(&Rect);
    State = BUTTON_IsPressed(pMsg->hWin);                                       // Get the state of the button and use it
    GUI_SetColor(_Colors.aColorBK[State]);                                      // as an index to choose a proper color (in this case the BK colors set are identical)
    GUI_FillRoundedRect(Rect.x0, Rect.y0, Rect.x1, Rect.y1, 5);                 // Fill a rounded rectangle with the color
    GUI_SetColor(_Colors.aColorFrame[State]);                                   // Choose a color for the button frame and the txt to be displayed
    GUI_DrawRoundedRect(Rect.x0, Rect.y0, Rect.x1, Rect.y1, 5);                 // Draw a rounded rectangle around the button
    GUI_DrawRoundedRect(Rect.x0 + 1, Rect.y0 + 1, Rect.x1 - 1, Rect.y1 - 1, 5); // And another one
    BUTTON_GetText(pMsg->hWin, acText, sizeof(acText));                         // Get Button text
    GUI_SetTextMode(GUI_TM_TRANS);
    Rect.x0 += 2;
    GUI_DispStringInRect(acText, &Rect, GUI_TA_HCENTER | GUI_TA_VCENTER);       // Display the button text inside the button rectangle
    break;
  default:
    BUTTON_Callback(pMsg);
    break;
  }
}

/*********************************************************************
*
*       _cbKeybord
*/
static void _cbKeybord(WM_MESSAGE * pMsg) {
  GUI_RECT Rect;
  WM_HWIN  hItem;
  WM_HWIN  hParent;
  int      NCode;
  int      Id;
  int      i;
  char     c;
  char     acTextSrc[100 + 1];
  char     acTextDest[100 + 1];
  int      Len;
  uint8_t uart_buffer[50]={0};

  switch (pMsg->MsgId) {
  case WM_INIT_DIALOG:
    //
    // Set a custom callback for each button
    //
    for (i = 0; i <= ID_BUTTON_28 - ID_BUTTON_0; i++) {
      hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_0 + i);
      WM_SetCallback(hItem, _cbButton);
    }
    break;
  case WM_PAINT:
    //
    // Fill background with black and draw a frame around the keyboard window
    //
    GUI_SetBkColor(GUI_BLACK);
    GUI_Clear();
    WM_GetClientRect(&Rect);
    GUI_SetColor(COLOR_BUTTON_FRAME_0);
    GUI_DrawRoundedRect(0, 0, Rect.x1, Rect.y1 + 5, 5);
    break;
  case WM_NOTIFY_PARENT:
    //
    // The important part
    //
    Id    = WM_GetId(pMsg->hWinSrc);
    NCode = pMsg->Data.v;
    sprintf(uart_buffer,"NCode: %d ",NCode);
    HAL_UART_Transmit(&huart2,uart_buffer,sizeof(uart_buffer),5);
    switch(NCode) {
    case WM_NOTIFICATION_RELEASED:
      //
      // Get the parent window of the keyboard (it's the main dialog with the edit widget)
      //
      hParent = WM_GetParent(pMsg->hWin);
      //
      // With the handle of the parent window we can get the edit handle by its ID
      //
      hItem   = WM_GetDialogItem(hParent, id_edit_0);
      //
      // Set Focus on the edit widget to make sure it gets the key input
      //
      WM_SetFocus(hItem);
      if (Id < ID_BUTTON_26) {
        //
        // With the ID of the pressed button calculate the character which should be displayed
        //
        c = 'a' + (Id - ID_BUTTON_0);
        //
        // Store key messages, important that we generate two since every key needs to be released, too
        //
        GUI_StoreKeyMsg(c, 1);
        GUI_StoreKeyMsg(c, 0);
      } else if (Id == ID_BUTTON_26) {  // Back button
        //
        // Get the text currently shown in the edit widget, manipulate it and write it back
        //
        EDIT_GetText(hItem, acTextSrc, sizeof(acTextSrc));  // Get
        Len = strlen(acTextSrc);
        if (Len > 0) {
          acTextSrc[Len - 1] = '\0';                        // Adapt
          EDIT_SetText(hItem, acTextSrc);                   // Write back
        }
      } else if (Id == ID_BUTTON_27) {                      // Clr button
        EDIT_SetText(hItem, "");                            // Simply set noting as text
      } else if (Id == ID_BUTTON_28) {                      // Space button
        c = ' ';                                            // Same as other characters
        GUI_StoreKeyMsg(c, 1);
        GUI_StoreKeyMsg(c, 0);
      }
      break;
    }
    break;
  default:
    WM_DefaultProc(pMsg);
    break;
  }
}

/*********************************************************************
*
*       _cbDialog
*/
static void _cbDialog(WM_MESSAGE * pMsg) {
  WM_HWIN hItem;
  int     NCode;
  int     Id;

  switch (pMsg->MsgId) {
  case WM_INIT_DIALOG:
    //
    // Initialization of 'Window'
    //
    hItem = pMsg->hWin;
    WINDOW_SetBkColor(hItem, GUI_MAKE_COLOR(0x00000000));
    //
    // Initialization of 'Edit'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_0);
    EDIT_SetText(hItem, "");
    break;
  case WM_TOUCH:
    //
    // If we touch somewhere on the screnn the keyboard gets hidden
    //
    WM_HideWindow(_hKeyboard);
    break;
  case WM_NOTIFY_PARENT:
    Id    = WM_GetId(pMsg->hWinSrc);
    NCode = pMsg->Data.v;
    switch(Id) {
    case ID_EDIT_0: // Notifications sent by 'Edit'
      switch(NCode) {
      case WM_NOTIFICATION_RELEASED:
        //
        // After a click inside the edit widget, we show the keyboard
        //
        WM_ShowWindow(_hKeyboard);
        break;
      }
      break;
    }
    break;
  default:
    WM_DefaultProc(pMsg);
    break;
  }
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       MainTask
*/
void createKeyboard(void) {
  int xSize, ySize;
  int xPos,  yPos;

  //
  // Create te main dialog containing an edit widget and a keyboard dialog
  //
  //
  // Get some properties and calculate the x and y position for the keaboard
  //
  xSize      = WM_GetWindowSizeX(hWin);
  ySize      = WM_GetWindowSizeY(hWin);
  xPos       = (xSize - ((BUTTON_X_SIZE + BUTTON_SPACING) * 10 + BUTTON_SPACING)) / 2;
  yPos       = ySize  - ((BUTTON_Y_SIZE + BUTTON_SPACING) *  3 + BUTTON_SPACING);
  //
  // Create the keyboard dialog as a child of the main dialog, but hide it for the beginning
  //
  _hKeyboard = GUI_CreateDialogBox(_aKeyboardDialogCreate, GUI_COUNTOF(_aKeyboardDialogCreate), _cbKeybord, hWin, xPos, yPos);
  WM_HideWindow(_hKeyboard);  // It's hidden

}
