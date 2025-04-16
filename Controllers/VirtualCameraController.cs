using Avalonia.Input;
using System;
using System.Numerics;

namespace RobotAIArm.Controllers
{
    public class VirtualCameraController
    {
        public Matrix4x4 ViewMatrix { get; private set; } = Matrix4x4.Identity;
        public Matrix4x4 ProjectionMatrix { get; private set; } = Matrix4x4.Identity;
        private Vector3 _cameraPosition = new Vector3(0, 0, 5);
        private float _yaw = 0f;
        private float _pitch = 0f;
        private float _moveSpeed = 40.0f;
        private float _rotationSpeed = 0.01f;
        private float _aspectRatio;
        public VirtualCameraController(float aspectRatio)
        {
            _aspectRatio = aspectRatio;
            UpdateViewMatrix();
            UpdateProjectionMatrix();
        }
        public void SetCameraPosition(Vector3 position)
        {
            _cameraPosition = position;
            UpdateViewMatrix();
        }
        public void SetCameraRotation(float yaw, float pitch)
        {
            _yaw = yaw;
            _pitch = pitch;
            UpdateViewMatrix();
        }
        public void Update(double deltaTime, InputState input)
        {
            float deltaSeconds = (float)deltaTime;
            // Calculate forward vector using spherical coordinates.
            float cosPitch = MathF.Cos(_pitch);
            Vector3 forward = new Vector3(
                cosPitch * MathF.Sin(_yaw),
                cosPitch * MathF.Cos(_yaw),
                MathF.Sin(_pitch)
            );
            forward = Vector3.Normalize(forward);

            // Use a fixed world up (Z-up).
            Vector3 worldUp = Vector3.UnitZ;
            // Right vector = normalized cross(forward, worldUp)
            Vector3 right = Vector3.Normalize(Vector3.Cross(forward, worldUp));

            // Move forward/backward.
            if (input.IsKeyDown(Key.W))
                _cameraPosition += forward * _moveSpeed * deltaSeconds;
            if (input.IsKeyDown(Key.S))
                _cameraPosition -= forward * _moveSpeed * deltaSeconds;
            // Strafe left/right.
            if (input.IsKeyDown(Key.A))
                _cameraPosition -= right * _moveSpeed * deltaSeconds;
            if (input.IsKeyDown(Key.D))
                _cameraPosition += right * _moveSpeed * deltaSeconds;

            if (input.IsMouseDown(MouseButton.Left))
            {
                _yaw += (float)input.MouseDelta.X * _rotationSpeed;
                _pitch -= (float)input.MouseDelta.Y * _rotationSpeed;
                _pitch = Math.Clamp(_pitch, -MathF.PI / 2f, MathF.PI / 2f);
            }
            UpdateViewMatrix();
        }

        public void UpdateAspectRatio(float aspectRatio)
        {
            _aspectRatio = aspectRatio;
            UpdateProjectionMatrix();
        }
        private void UpdateViewMatrix()
        {
            // Compute the forward vector from yaw and pitch.
            // Here we assume:
            // - yaw = 0 means looking along +Y.
            // - pitch = 0 means level.
            float cosPitch = MathF.Cos(_pitch);
            Vector3 forward = new Vector3(
                cosPitch * MathF.Sin(_yaw),   // X component
                cosPitch * MathF.Cos(_yaw),   // Y component
                MathF.Sin(_pitch)             // Z component
            );
            Vector3 cameraTarget = _cameraPosition + forward;
            // Use a fixed world up vector (Z-up)
            Vector3 cameraUp = Vector3.UnitZ;

            ViewMatrix = Matrix4x4.CreateLookAt(_cameraPosition, cameraTarget, cameraUp);
            //Console.WriteLine($"yaw: {_yaw}, pitch: {_pitch}");
            //Console.WriteLine($"CameraPosition: {_cameraPosition}, CameraTarget: {cameraTarget}, CameraUp: {cameraUp}");
        }

        private void UpdateProjectionMatrix()
        {
            // Use a 60° FOV and far clip of 1000.
            ProjectionMatrix = Matrix4x4.CreatePerspectiveFieldOfView(MathF.PI / 4, _aspectRatio, 0.5f, 1000f);
        }
        public Matrix4x4[] GetMVPMatrices(Matrix4x4 modelMatrix)
        {
            return new Matrix4x4[] { modelMatrix, ViewMatrix, ProjectionMatrix };
        }
    }
}
