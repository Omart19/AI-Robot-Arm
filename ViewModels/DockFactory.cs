using System;
using System.Collections.Generic;
using RobotAIArm.Models.Documents;
using RobotAIArm.Models.Tools;
using RobotAIArm.ViewModels.Tools;
using RobotAIArm.ViewModels.Views;
using Dock.Avalonia.Controls;
using Dock.Model.Controls;
using Dock.Model.Core;
using Dock.Model.Mvvm;
using Dock.Model.Mvvm.Controls;

namespace RobotAIArm.ViewModels;

public class DockFactory : Factory
{
    private readonly object _context;
    private IRootDock? _rootDock;
    public DockFactory(object context)
    {
        _context = context;
    }


    public override IRootDock CreateLayout()
    {
        
        var tool1 = new Tool1ViewModel {Id = "Tool1", Title = "Camera View"};
        var tool2 = new Tool2ViewModel {Id = "Tool2", Title = "Tool2"};
        var tool3 = new Tool3ViewModel {Id = "Tool3", Title = "Tool3"};
        var tool4 = new Tool4ViewModel {Id = "Tool4", Title = "Tool4"};
        var tool5 = new Tool5ViewModel {Id = "Tool5", Title = "3D View"};

        var leftDock = new ProportionalDock
        {
            Proportion = 0.1,
            Orientation = Orientation.Vertical,
            ActiveDockable = null,
            VisibleDockables = CreateList<IDockable>
            (
                new ToolDock
                {
                    ActiveDockable = tool2,
                    VisibleDockables = CreateList<IDockable>(tool2),
                    Alignment = Alignment.Left
                }
                
            )
        };

        var rightDock = new ProportionalDock
        {
            Proportion = 0.6,
            Orientation = Orientation.Vertical,
            ActiveDockable = null,
            VisibleDockables = CreateList<IDockable>
            (
                new ProportionalDock
                {
                    Proportion = 0.7,
                    Orientation = Orientation.Horizontal,
                    ActiveDockable = null,
                    VisibleDockables = CreateList<IDockable>
                    (
                        new ToolDock
                        {
                            ActiveDockable = tool5,
                            VisibleDockables = CreateList<IDockable>(tool5),
                            Alignment = Alignment.Left,
                            GripMode = GripMode.Hidden
                        },
                        new ProportionalDockSplitter(),
                        new ToolDock
                        {
                            ActiveDockable = tool1,
                            VisibleDockables = CreateList<IDockable>(tool1),
                            Alignment = Alignment.Right
                        }
                    )
                },
                new ProportionalDockSplitter(),
                
                new ToolDock
                {
                    ActiveDockable = tool3,
                    VisibleDockables = CreateList<IDockable>(tool3, tool4),
                    Alignment = Alignment.Bottom
                }
            )
        };

        // var bottomrightDock = new ProportionalDock
        // {
        //     Proportion = 0.25,
        //     Orientation = Orientation.Horizontal,
        //     ActiveDockable = null,
        //     VisibleDockables = CreateList<IDockable>
        //     (
        //         new ToolDock
        //         {
        //             ActiveDockable = tool3,
        //             VisibleDockables = CreateList<IDockable>(tool3, tool4),
        //             Alignment = Alignment.Bottom
        //         }
        //     )
        // };

        var mainLayout = new ProportionalDock
        {
            Orientation = Orientation.Horizontal,
            VisibleDockables = CreateList<IDockable>
            (
                leftDock,
                new ProportionalDockSplitter(),
                rightDock
            )
        };

        var dashboardView = new DashboardViewModel
        {
            Id = "Dashboard",
            Title = "Dashboard"
        };

        var homeView = new HomeViewModel
        {
            Id = "Home",
            Title = "Home",
            ActiveDockable = mainLayout,
            VisibleDockables = CreateList<IDockable>(mainLayout)
        };

        var rootDock = CreateRootDock();

        rootDock.IsCollapsable = false;
        rootDock.ActiveDockable = dashboardView;
        rootDock.DefaultDockable = homeView;
        rootDock.VisibleDockables = CreateList<IDockable>(dashboardView, homeView);

        _rootDock = rootDock;
            
        return rootDock;
    }

    public override IDockWindow? CreateWindowFrom(IDockable dockable)
    {
        var window = base.CreateWindowFrom(dockable);

        if (window != null)
        {
            window.Title = "AI Robot arm Dashboard";
        }
        return window;
    }

    public override void InitLayout(IDockable layout)
    {
        ContextLocator = new Dictionary<string, Func<object?>>
        {
            
            ["Tool1"] = () => new Tool1(),
            ["Tool2"] = () => new Tool2(),
            ["Tool3"] = () => new Tool3(),
            ["Tool4"] = () => new Tool4(),
            ["Tool5"] = () => new Tool5(),
            ["Dashboard"] = () => layout,
            ["Home"] = () => _context
        };

        DockableLocator = new Dictionary<string, Func<IDockable?>>()
        {
            ["Root"] = () => _rootDock
        };

        HostWindowLocator = new Dictionary<string, Func<IHostWindow?>>
        {
            [nameof(IDockWindow)] = () => new HostWindow()
        };

        base.InitLayout(layout);
    }
}
