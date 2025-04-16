using Avalonia;

namespace RobotAIArm.Themes;

public interface IThemeManager
{
    void Initialize(Application application);

    void Switch(int index);
}
