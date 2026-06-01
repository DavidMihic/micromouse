/*
 * maze_solver.h
 *
 *  Created on: May 30, 2026
 *      Author: andrija
 */

#ifndef INC_MAZE_SOLVER_H_
#define INC_MAZE_SOLVER_H_

#include <stdint.h>
#include <stdbool.h>

#define MAZE_SIZE       16u
#define MAZE_MAX_STACK  (MAZE_SIZE * MAZE_SIZE)

typedef enum
{
    MAZE_DIR_NORTH = 0,
    MAZE_DIR_EAST  = 1,
    MAZE_DIR_SOUTH = 2,
    MAZE_DIR_WEST  = 3
} MazeDirection;

typedef enum
{
    MAZE_SOLVER_STOPPED = 0,
    MAZE_SOLVER_RUNNING,
    MAZE_SOLVER_REACHED_GOAL,
    MAZE_SOLVER_RETURNED_TO_START,
    MAZE_SOLVER_FAST_RUN_DONE,
    MAZE_SOLVER_FULLY_EXPLORED,
    MAZE_SOLVER_ERROR,
    MAZE_SOLVER_ABORTED
} MazeSolverStatus;

typedef enum
{
    MAZE_RUN_IDLE = 0,
    MAZE_RUN_EXPLORE,
    MAZE_RUN_RETURN_TO_START,
    MAZE_RUN_FAST
} MazeRunMode;

typedef struct
{
    bool left;
    bool front;
    bool right;
} MazeWallReading;

typedef void (*MazeBeginWallReadFn)(void *user);
typedef bool (*MazeReadWallsFn)(MazeWallReading *walls, void *user);
typedef bool (*MazeStartMoveCellFn)(void *user);
typedef bool (*MazeStartMoveCellsFn)(uint16_t cells, void *user);
typedef bool (*MazeStartTurnFn)(int16_t degrees, void *user);
typedef bool (*MazeMotionDoneFn)(void *user);
typedef void (*MazeStopMotionFn)(void *user);
typedef void (*MazeLogFn)(const char *msg, void *user);

typedef struct
{
    MazeBeginWallReadFn begin_wall_read;
    MazeReadWallsFn read_walls;

    MazeStartMoveCellFn start_move_cell;
    MazeStartMoveCellsFn start_move_cells;
    MazeStartTurnFn start_turn_deg;
    MazeMotionDoneFn motion_done;
    MazeStopMotionFn stop_motion;

    MazeLogFn log;
    void *user;

    uint8_t start_x;
    uint8_t start_y;
    MazeDirection start_dir;

    bool left_bias;

    /* If true, FAST run combines consecutive straight cells into one longer
     * straight move. Exploration and return-to-start stay cell-by-cell.
     */
    bool fast_run_join_straights;
} MazeSolverConfig;

typedef struct
{
    uint8_t walls;
    uint8_t known;
    uint8_t visited;
} MazeCell;

typedef struct
{
    uint8_t x;
    uint8_t y;
} MazePosition;

typedef enum
{
    MAZE_STATE_IDLE = 0,
    MAZE_STATE_BEGIN_CELL,
    MAZE_STATE_WAIT_WALLS,
    MAZE_STATE_ROUTE_START_STEP,
    MAZE_STATE_START_TURN,
    MAZE_STATE_WAIT_TURN,
    MAZE_STATE_START_MOVE,
    MAZE_STATE_WAIT_MOVE,
    MAZE_STATE_FINISHED,
    MAZE_STATE_ERROR
} MazeInternalState;

typedef struct
{
    MazeSolverConfig cfg;

    MazeCell cells[MAZE_SIZE][MAZE_SIZE];

    MazePosition stack[MAZE_MAX_STACK];
    uint16_t stack_size;

    MazeDirection route_dirs[MAZE_MAX_STACK];
    uint16_t route_length;
    uint16_t route_index;

    uint8_t x;
    uint8_t y;
    MazeDirection dir;

    MazeDirection target_dir;
    uint16_t move_cell_count;

    bool backtracking;

    uint32_t steps;

    MazeRunMode run_mode;
    MazeInternalState state;
    MazeSolverStatus status;
} MazeSolver;

void MazeSolver_Init(MazeSolver *solver, const MazeSolverConfig *cfg);
void MazeSolver_Start(MazeSolver *solver);
void MazeSolver_Stop(MazeSolver *solver);

MazeSolverStatus MazeSolver_Task(MazeSolver *solver);

bool MazeSolver_StartReturnToStart(MazeSolver *solver);
bool MazeSolver_StartFastRun(MazeSolver *solver);

uint8_t MazeSolver_GetX(const MazeSolver *solver);
uint8_t MazeSolver_GetY(const MazeSolver *solver);
MazeDirection MazeSolver_GetDir(const MazeSolver *solver);
MazeSolverStatus MazeSolver_GetStatus(const MazeSolver *solver);
MazeRunMode MazeSolver_GetRunMode(const MazeSolver *solver);
uint16_t MazeSolver_GetRouteLength(const MazeSolver *solver);
uint16_t MazeSolver_GetRouteIndex(const MazeSolver *solver);

#endif /* INC_MAZE_SOLVER_H_ */
