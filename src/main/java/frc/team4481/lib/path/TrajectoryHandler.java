package frc.team4481.lib.path;

import com.pathplanner.lib.path.PathPlannerPath;

import java.util.HashMap;


/**
 * Utility class to load and transform a {@code PathPlannerPath}.
 */
public class TrajectoryHandler {
    private static TrajectoryHandler instance;

    private PathPlannerPath currentPath;
    private final HashMap<String, PathPlannerPath> loadedPathMap = new HashMap<>();

    private TrajectoryHandler() {}

    /**
     * Gets the {@code TrajectoryHandler} instance.
     *
     * @return singleton instance of the {@code TrajectoryHandler}
     */
    public static TrajectoryHandler getInstance() {
        if (instance == null)
            instance = new TrajectoryHandler();

        return instance;
    }

    /**
     * Preloads a Path Planner 2 path for faster processing.
     * This method is intended to be called in the disabled state of the robot.
     *
     * @param pathName      Filename of the path minus file extension
     */
    public void preloadPath(
            String pathName
    ) {
        loadedPathMap.put(
                pathName,
                PathPlannerPath.fromPathFile(pathName)
        );
    }

    /**
     * Gets the current {@code PathPlannerPath}.
     *
     * @return current {@code PathPlannerPath}
     */
    public PathPlannerPath getCurrentPath() {
        return currentPath;
    }

    /**
     * Transforms a Path Planner 2 path into a {@code PathPlannerPath}. Preloading a path increases loading speed
     *
     * @param pathName      Filename of the path minus file extension
     *
     * @see TrajectoryHandler#preloadPath(String) preloading trajectories
     */
    public PathPlannerPath setPath(
            String pathName
    ) {
        try {
            if (loadedPathMap.containsKey(pathName))
                return setPreloadedPath(pathName);
            return setUnloadedPath(pathName);
        } catch (PathNotLoadedException e) {
            e.printStackTrace();
        }

        return null;
    }

    /**
     * Sets a preloaded {@code PathPlannerPath} as current trajectory.
     *
     * @param pathName name of the path as noted in Path Planner 2
     * @throws PathNotLoadedException if the path has not been loaded yet
     * @return the currently loaded trajectory
     */
    private PathPlannerPath setPreloadedPath(String pathName) throws PathNotLoadedException {
        if (!loadedPathMap.containsKey(pathName))
            throw new PathNotLoadedException("Path '" + pathName + "' has not been preloaded yet.");

        currentPath = loadedPathMap.get(pathName);

        return currentPath;
    }

    /**
     * Sets an unloaded {@code PathPlannerTrajectory} as current trajectory and loads it for future use.
     *
     * @param pathName name of the path as noted in Path Planner 2
     * @throws PathNotLoadedException if the path has not been loaded yet
     * @return the currently loaded trajectory
     */
    private PathPlannerPath setUnloadedPath(
            String pathName
    ) throws PathNotLoadedException {
        preloadPath(pathName);

        return setPreloadedPath(pathName);
    }



}
