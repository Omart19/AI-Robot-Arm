//InputState.cs
using Avalonia;
using Avalonia.Input;

namespace RobotAIArm.Controllers
{
    //=================================================================
    // InputState.cs (your provided version)
    //=================================================================
    public class InputState
    {
        public Point MouseDelta { get; set; }
        private readonly bool[] _mouseDown = new bool[256];
        private readonly bool[] _keyDown = new bool[256];

        public bool IsMouseDown(MouseButton button) => _mouseDown[(int)button];
        public bool IsKeyDown(Key key) => _keyDown[(int)key];
        public void SetMouseDown(MouseButton button, bool pressed) => _mouseDown[(int)button] = pressed;
        public void SetKeyDown(Key key, bool pressed) => _keyDown[(int)key] = pressed;
        public void ClearDelta() => MouseDelta = new Point(0, 0);
    }



}