/*
 * maze_solver.c
 *
 *  Created on: May 30, 2026
 *      Author: andrija
 */

#include "maze_solver.h"
#include <string.h>

#define WALL_BIT(dir) ((uint8_t)(1u << (uint8_t)(dir)))
#define BFS_INVALID  255u

static MazeDirection Dir_Left(MazeDirection dir)
{
    return (MazeDirection)(((uint8_t)dir + 3u) & 3u);
}

static MazeDirection Dir_Right(MazeDirection dir)
{
    return (MazeDirection)(((uint8_t)dir + 1u) & 3u);
}

static MazeDirection Dir_Back(MazeDirection dir)
{
    return (MazeDirection)(((uint8_t)dir + 2u) & 3u);
}

static MazeDirection Dir_Opposite(MazeDirection dir)
{
    return Dir_Back(dir);
}

static int DirDx(MazeDirection dir)
{
    if (dir == MAZE_DIR_EAST)
        return 1;

    if (dir == MAZE_DIR_WEST)
        return -1;

    return 0;
}

static int DirDy(MazeDirection dir)
{
    if (dir == MAZE_DIR_NORTH)
        return 1;

    if (dir == MAZE_DIR_SOUTH)
        return -1;

    return 0;
}

static bool IsInside(int x, int y)
{
    return (x >= 0) &&
           (x < (int)MAZE_SIZE) &&
           (y >= 0) &&
           (y < (int)MAZE_SIZE);
}

static bool IsGoalCell(uint8_t x, uint8_t y)
{
    return ((x == 7u || x == 8u) &&
            (y == 7u || y == 8u));
}

static void LogMsg(MazeSolver *solver, const char *msg)
{
    if ((solver->cfg.log != 0) && (msg != 0))
        solver->cfg.log(msg, solver->cfg.user);
}

static void SetWall(MazeSolver *solver,
                    uint8_t x,
                    uint8_t y,
                    MazeDirection dir,
                    bool wall)
{
    MazeCell *cell = &solver->cells[y][x];
    uint8_t bit = WALL_BIT(dir);

    cell->known |= bit;

    if (wall)
        cell->walls |= bit;
    else
        cell->walls &= (uint8_t)~bit;

    int nx = (int)x + DirDx(dir);
    int ny = (int)y + DirDy(dir);

    if (IsInside(nx, ny))
    {
        MazeDirection opposite = Dir_Opposite(dir);
        MazeCell *neighbor = &solver->cells[ny][nx];
        uint8_t opposite_bit = WALL_BIT(opposite);

        neighbor->known |= opposite_bit;

        if (wall)
            neighbor->walls |= opposite_bit;
        else
            neighbor->walls &= (uint8_t)~opposite_bit;
    }
}

static bool IsKnownOpen(const MazeSolver *solver,
                        uint8_t x,
                        uint8_t y,
                        MazeDirection dir)
{
    const MazeCell *cell = &solver->cells[y][x];
    uint8_t bit = WALL_BIT(dir);

    if ((cell->known & bit) == 0u)
        return false;

    return ((cell->walls & bit) == 0u);
}

static bool NeighborOf(uint8_t x,
                       uint8_t y,
                       MazeDirection dir,
                       uint8_t *nx,
                       uint8_t *ny)
{
    int tx = (int)x + DirDx(dir);
    int ty = (int)y + DirDy(dir);

    if (!IsInside(tx, ty))
        return false;

    *nx = (uint8_t)tx;
    *ny = (uint8_t)ty;

    return true;
}

static void InitOuterWalls(MazeSolver *solver)
{
    for (uint8_t x = 0u; x < MAZE_SIZE; x++)
    {
        SetWall(solver, x, 0u, MAZE_DIR_SOUTH, true);
        SetWall(solver, x, MAZE_SIZE - 1u, MAZE_DIR_NORTH, true);
    }

    for (uint8_t y = 0u; y < MAZE_SIZE; y++)
    {
        SetWall(solver, 0u, y, MAZE_DIR_WEST, true);
        SetWall(solver, MAZE_SIZE - 1u, y, MAZE_DIR_EAST, true);
    }
}

static void MarkVisited(MazeSolver *solver)
{
    MazeCell *cell = &solver->cells[solver->y][solver->x];

    if (cell->visited < 255u)
        cell->visited++;
}

static bool ApplyWallReading(MazeSolver *solver, const MazeWallReading *walls)
{
    if (walls == 0)
        return false;

    SetWall(solver, solver->x, solver->y, Dir_Left(solver->dir), walls->left);
    SetWall(solver, solver->x, solver->y, solver->dir, walls->front);
    SetWall(solver, solver->x, solver->y, Dir_Right(solver->dir), walls->right);

    return true;
}

static bool PushCurrentPosition(MazeSolver *solver)
{
    if (solver->stack_size >= MAZE_MAX_STACK)
        return false;

    solver->stack[solver->stack_size].x = solver->x;
    solver->stack[solver->stack_size].y = solver->y;
    solver->stack_size++;

    return true;
}

static bool FindUnvisitedOpenNeighbor(MazeSolver *solver, MazeDirection *out_dir)
{
    MazeDirection candidates[4];

    if (solver->cfg.left_bias)
    {
        candidates[0] = Dir_Left(solver->dir);
        candidates[1] = solver->dir;
        candidates[2] = Dir_Right(solver->dir);
        candidates[3] = Dir_Back(solver->dir);
    }
    else
    {
        candidates[0] = Dir_Right(solver->dir);
        candidates[1] = solver->dir;
        candidates[2] = Dir_Left(solver->dir);
        candidates[3] = Dir_Back(solver->dir);
    }

    for (uint8_t i = 0u; i < 4u; i++)
    {
        MazeDirection dir = candidates[i];

        if (!IsKnownOpen(solver, solver->x, solver->y, dir))
            continue;

        uint8_t nx;
        uint8_t ny;

        if (!NeighborOf(solver->x, solver->y, dir, &nx, &ny))
            continue;

        if (solver->cells[ny][nx].visited == 0u)
        {
            *out_dir = dir;
            return true;
        }
    }

    return false;
}

static bool GetBacktrackDirection(MazeSolver *solver, MazeDirection *out_dir)
{
    if (solver->stack_size == 0u)
        return false;

    MazePosition target = solver->stack[solver->stack_size - 1u];

    int dx = (int)target.x - (int)solver->x;
    int dy = (int)target.y - (int)solver->y;

    if ((dx == 1) && (dy == 0))
        *out_dir = MAZE_DIR_EAST;
    else if ((dx == -1) && (dy == 0))
        *out_dir = MAZE_DIR_WEST;
    else if ((dx == 0) && (dy == 1))
        *out_dir = MAZE_DIR_NORTH;
    else if ((dx == 0) && (dy == -1))
        *out_dir = MAZE_DIR_SOUTH;
    else
        return false;

    return true;
}

static int16_t RelativeTurnDeg(MazeDirection current, MazeDirection target)
{
    uint8_t diff = ((uint8_t)target - (uint8_t)current) & 3u;

    if (diff == 0u)
        return 0;

    if (diff == 1u)
        return -90;

    if (diff == 2u)
        return 180;

    return 90;
}

static bool UpdateLogicalPosition(MazeSolver *solver)
{
    int nx = (int)solver->x + DirDx(solver->dir);
    int ny = (int)solver->y + DirDy(solver->dir);

    if (!IsInside(nx, ny))
        return false;

    solver->x = (uint8_t)nx;
    solver->y = (uint8_t)ny;
    solver->steps++;

    return true;
}

static bool UpdateLogicalPositionCells(MazeSolver *solver, uint16_t cells)
{
    if ((solver == 0) || (cells == 0u))
        return false;

    for (uint16_t i = 0u; i < cells; i++)
    {
        if (!UpdateLogicalPosition(solver))
            return false;
    }

    return true;
}

static bool RouteIsOpenForCells(const MazeSolver *solver, uint16_t cells)
{
    if ((solver == 0) || (cells == 0u))
        return false;

    uint8_t x = solver->x;
    uint8_t y = solver->y;

    for (uint16_t i = 0u; i < cells; i++)
    {
        uint8_t nx;
        uint8_t ny;

        if (!IsKnownOpen(solver, x, y, solver->dir))
            return false;

        if (!NeighborOf(x, y, solver->dir, &nx, &ny))
            return false;

        x = nx;
        y = ny;
    }

    return true;
}

static uint16_t CountFastStraightCells(const MazeSolver *solver)
{
    if (solver == 0)
        return 1u;

    if (solver->run_mode != MAZE_RUN_FAST)
        return 1u;

    if (!solver->cfg.fast_run_join_straights)
        return 1u;

    if (solver->route_index >= solver->route_length)
        return 1u;

    uint16_t count = 1u;

    while ((solver->route_index + count) < solver->route_length)
    {
        if (solver->route_dirs[solver->route_index + count] != solver->dir)
            break;

        if (count >= MAZE_MAX_STACK)
            break;

        count++;
    }

    return count;
}

static void FinishWithStatus(MazeSolver *solver, MazeSolverStatus status)
{
    solver->status = status;
    solver->state = MAZE_STATE_FINISHED;

    if ((status == MAZE_SOLVER_REACHED_GOAL) ||
        (status == MAZE_SOLVER_RETURNED_TO_START) ||
        (status == MAZE_SOLVER_FAST_RUN_DONE) ||
        (status == MAZE_SOLVER_FULLY_EXPLORED) ||
        (status == MAZE_SOLVER_ERROR))
    {
        if (solver->cfg.stop_motion != 0)
            solver->cfg.stop_motion(solver->cfg.user);
    }
}

typedef bool (*MazeTargetPredicate)(const MazeSolver *solver,
                                    uint8_t x,
                                    uint8_t y,
                                    void *ctx);

static bool TargetIsStart(const MazeSolver *solver,
                          uint8_t x,
                          uint8_t y,
                          void *ctx)
{
    (void)ctx;

    return (x == solver->cfg.start_x) && (y == solver->cfg.start_y);
}

static bool TargetIsGoal(const MazeSolver *solver,
                         uint8_t x,
                         uint8_t y,
                         void *ctx)
{
    (void)solver;
    (void)ctx;

    return IsGoalCell(x, y);
}

static bool PlanShortestKnownRoute(MazeSolver *solver,
                                   uint8_t start_x,
                                   uint8_t start_y,
                                   MazeTargetPredicate target_predicate,
                                   void *ctx)
{
    static uint8_t seen[MAZE_SIZE][MAZE_SIZE];
    static uint8_t prev_x[MAZE_SIZE][MAZE_SIZE];
    static uint8_t prev_y[MAZE_SIZE][MAZE_SIZE];
    static uint8_t prev_dir[MAZE_SIZE][MAZE_SIZE];
    static MazePosition queue[MAZE_MAX_STACK];

    if ((solver == 0) || (target_predicate == 0))
        return false;

    memset(seen, 0, sizeof(seen));
    memset(prev_x, BFS_INVALID, sizeof(prev_x));
    memset(prev_y, BFS_INVALID, sizeof(prev_y));
    memset(prev_dir, BFS_INVALID, sizeof(prev_dir));

    solver->route_length = 0u;
    solver->route_index = 0u;

    if (!IsInside(start_x, start_y))
        return false;

    if (target_predicate(solver, start_x, start_y, ctx))
        return true;

    uint16_t head = 0u;
    uint16_t tail = 0u;

    queue[tail].x = start_x;
    queue[tail].y = start_y;
    tail++;

    seen[start_y][start_x] = 1u;

    bool found = false;
    uint8_t target_x = start_x;
    uint8_t target_y = start_y;

    while (head < tail)
    {
        MazePosition current = queue[head];
        head++;

        for (uint8_t d = 0u; d < 4u; d++)
        {
            MazeDirection dir = (MazeDirection)d;
            uint8_t nx;
            uint8_t ny;

            if (!IsKnownOpen(solver, current.x, current.y, dir))
                continue;

            if (!NeighborOf(current.x, current.y, dir, &nx, &ny))
                continue;

            if (seen[ny][nx] != 0u)
                continue;

            seen[ny][nx] = 1u;
            prev_x[ny][nx] = current.x;
            prev_y[ny][nx] = current.y;
            prev_dir[ny][nx] = (uint8_t)dir;

            if (target_predicate(solver, nx, ny, ctx))
            {
                target_x = nx;
                target_y = ny;
                found = true;
                head = tail; /* finish after this loop */
                break;
            }

            if (tail >= MAZE_MAX_STACK)
                return false;

            queue[tail].x = nx;
            queue[tail].y = ny;
            tail++;
        }
    }

    if (!found)
        return false;

    uint8_t cx = target_x;
    uint8_t cy = target_y;
    uint16_t length = 0u;

    while (!((cx == start_x) && (cy == start_y)))
    {
        if (length >= MAZE_MAX_STACK)
            return false;

        uint8_t d = prev_dir[cy][cx];
        uint8_t px = prev_x[cy][cx];
        uint8_t py = prev_y[cy][cx];

        if ((d == BFS_INVALID) || (px == BFS_INVALID) || (py == BFS_INVALID))
            return false;

        solver->route_dirs[length] = (MazeDirection)d;
        length++;

        cx = px;
        cy = py;
    }

    for (uint16_t i = 0u; i < (length / 2u); i++)
    {
        MazeDirection tmp = solver->route_dirs[i];
        solver->route_dirs[i] = solver->route_dirs[length - 1u - i];
        solver->route_dirs[length - 1u - i] = tmp;
    }

    solver->route_length = length;
    solver->route_index = 0u;

    return true;
}

static void StartPlannedRoute(MazeSolver *solver, MazeRunMode mode)
{
    solver->run_mode = mode;
    solver->status = MAZE_SOLVER_RUNNING;
    solver->move_cell_count = 1u;

    if (solver->route_length == 0u)
    {
        if (mode == MAZE_RUN_RETURN_TO_START)
            FinishWithStatus(solver, MAZE_SOLVER_RETURNED_TO_START);
        else if (mode == MAZE_RUN_FAST)
            FinishWithStatus(solver, MAZE_SOLVER_FAST_RUN_DONE);
        else
            FinishWithStatus(solver, MAZE_SOLVER_ERROR);

        return;
    }

    solver->state = MAZE_STATE_ROUTE_START_STEP;
}

static void StartNextRouteStep(MazeSolver *solver)
{
    if (solver->route_index >= solver->route_length)
    {
        if (solver->run_mode == MAZE_RUN_RETURN_TO_START)
        {
            LogMsg(solver, "Returned to start");
            FinishWithStatus(solver, MAZE_SOLVER_RETURNED_TO_START);
        }
        else if (solver->run_mode == MAZE_RUN_FAST)
        {
            LogMsg(solver, "Fast run finished");
            FinishWithStatus(solver, MAZE_SOLVER_FAST_RUN_DONE);
        }
        else
            FinishWithStatus(solver, MAZE_SOLVER_ERROR);

        return;
    }

    solver->target_dir = solver->route_dirs[solver->route_index];
    solver->move_cell_count = 1u;
    solver->backtracking = false;
    solver->state = MAZE_STATE_START_TURN;
}

void MazeSolver_Init(MazeSolver *solver, const MazeSolverConfig *cfg)
{
    if ((solver == 0) || (cfg == 0))
        return;

    memset(solver, 0, sizeof(*solver));

    solver->cfg = *cfg;

    solver->x = cfg->start_x;
    solver->y = cfg->start_y;
    solver->dir = cfg->start_dir;
    solver->target_dir = cfg->start_dir;
    solver->move_cell_count = 1u;

    solver->run_mode = MAZE_RUN_IDLE;
    solver->state = MAZE_STATE_IDLE;
    solver->status = MAZE_SOLVER_STOPPED;

    InitOuterWalls(solver);
}

void MazeSolver_Start(MazeSolver *solver)
{
    if (solver == 0)
        return;

    solver->run_mode = MAZE_RUN_EXPLORE;
    solver->route_length = 0u;
    solver->route_index = 0u;
    solver->move_cell_count = 1u;
    solver->state = MAZE_STATE_BEGIN_CELL;
    solver->status = MAZE_SOLVER_RUNNING;
}

void MazeSolver_Stop(MazeSolver *solver)
{
    if (solver == 0)
        return;

    if (solver->cfg.stop_motion != 0)
        solver->cfg.stop_motion(solver->cfg.user);

    solver->run_mode = MAZE_RUN_IDLE;
    solver->state = MAZE_STATE_IDLE;
    solver->status = MAZE_SOLVER_ABORTED;
}

bool MazeSolver_StartReturnToStart(MazeSolver *solver)
{
    if (solver == 0)
        return false;

    if (!PlanShortestKnownRoute(solver,
                                solver->x,
                                solver->y,
                                TargetIsStart,
                                0))
    {
        LogMsg(solver, "No known route back to start");
        FinishWithStatus(solver, MAZE_SOLVER_ERROR);
        return false;
    }

    LogMsg(solver, "Returning to start");
    StartPlannedRoute(solver, MAZE_RUN_RETURN_TO_START);
    return (solver->status == MAZE_SOLVER_RUNNING) ||
           (solver->status == MAZE_SOLVER_RETURNED_TO_START);
}

bool MazeSolver_StartFastRun(MazeSolver *solver)
{
    if (solver == 0)
        return false;

    if (!PlanShortestKnownRoute(solver,
                                solver->x,
                                solver->y,
                                TargetIsGoal,
                                0))
    {
        LogMsg(solver, "No known fastest route to goal");
        FinishWithStatus(solver, MAZE_SOLVER_ERROR);
        return false;
    }

    LogMsg(solver, "Starting fastest run");
    StartPlannedRoute(solver, MAZE_RUN_FAST);
    return (solver->status == MAZE_SOLVER_RUNNING) ||
           (solver->status == MAZE_SOLVER_FAST_RUN_DONE);
}

MazeSolverStatus MazeSolver_Task(MazeSolver *solver)
{
    if (solver == 0)
        return MAZE_SOLVER_ERROR;

    if (solver->status != MAZE_SOLVER_RUNNING)
        return solver->status;

    switch (solver->state)
    {
        case MAZE_STATE_BEGIN_CELL:
        {
            if (solver->run_mode != MAZE_RUN_EXPLORE)
            {
                FinishWithStatus(solver, MAZE_SOLVER_ERROR);
                break;
            }

            if (IsGoalCell(solver->x, solver->y))
            {
                LogMsg(solver, "Goal reached");
                FinishWithStatus(solver, MAZE_SOLVER_REACHED_GOAL);
                break;
            }

            if (solver->cfg.begin_wall_read != 0)
                solver->cfg.begin_wall_read(solver->cfg.user);

            solver->state = MAZE_STATE_WAIT_WALLS;
            break;
        }

        case MAZE_STATE_WAIT_WALLS:
        {
            MazeWallReading walls;

            if (solver->run_mode != MAZE_RUN_EXPLORE)
            {
                FinishWithStatus(solver, MAZE_SOLVER_ERROR);
                break;
            }

            if (solver->cfg.read_walls == 0)
            {
                FinishWithStatus(solver, MAZE_SOLVER_ERROR);
                break;
            }

            if (!solver->cfg.read_walls(&walls, solver->cfg.user))
                break;

            MarkVisited(solver);

            if (!ApplyWallReading(solver, &walls))
            {
                FinishWithStatus(solver, MAZE_SOLVER_ERROR);
                break;
            }

            MazeDirection next_dir;

            if (FindUnvisitedOpenNeighbor(solver, &next_dir))
            {
                if (!PushCurrentPosition(solver))
                {
                    FinishWithStatus(solver, MAZE_SOLVER_ERROR);
                    break;
                }

                solver->target_dir = next_dir;
                solver->backtracking = false;
                solver->state = MAZE_STATE_START_TURN;
            }
            else if (GetBacktrackDirection(solver, &next_dir))
            {
                solver->target_dir = next_dir;
                solver->backtracking = true;
                solver->state = MAZE_STATE_START_TURN;
            }
            else
            {
                LogMsg(solver, "Maze fully explored, goal not found");
                FinishWithStatus(solver, MAZE_SOLVER_FULLY_EXPLORED);
            }

            break;
        }

        case MAZE_STATE_ROUTE_START_STEP:
        {
            StartNextRouteStep(solver);
            break;
        }

        case MAZE_STATE_START_TURN:
        {
            int16_t turn_deg = RelativeTurnDeg(solver->dir, solver->target_dir);

            if (turn_deg == 0)
            {
                solver->dir = solver->target_dir;
                solver->state = MAZE_STATE_START_MOVE;
                break;
            }

            if (solver->cfg.start_turn_deg == 0)
            {
                FinishWithStatus(solver, MAZE_SOLVER_ERROR);
                break;
            }

            if (!solver->cfg.start_turn_deg(turn_deg, solver->cfg.user))
            {
                FinishWithStatus(solver, MAZE_SOLVER_ERROR);
                break;
            }

            solver->state = MAZE_STATE_WAIT_TURN;
            break;
        }

        case MAZE_STATE_WAIT_TURN:
        {
            if (solver->cfg.motion_done == 0)
            {
                FinishWithStatus(solver, MAZE_SOLVER_ERROR);
                break;
            }

            if (!solver->cfg.motion_done(solver->cfg.user))
                break;

            solver->dir = solver->target_dir;
            solver->state = MAZE_STATE_START_MOVE;
            break;
        }

        case MAZE_STATE_START_MOVE:
        {
            uint16_t cells_to_move = CountFastStraightCells(solver);

            if (!RouteIsOpenForCells(solver, cells_to_move))
            {
                FinishWithStatus(solver, MAZE_SOLVER_ERROR);
                break;
            }

            solver->move_cell_count = cells_to_move;

            bool started = false;

            if (solver->cfg.start_move_cells != 0)
            {
                started = solver->cfg.start_move_cells(cells_to_move, solver->cfg.user);
            }
            else if ((cells_to_move == 1u) && (solver->cfg.start_move_cell != 0))
            {
                started = solver->cfg.start_move_cell(solver->cfg.user);
            }

            if (!started)
            {
                FinishWithStatus(solver, MAZE_SOLVER_ERROR);
                break;
            }

            solver->state = MAZE_STATE_WAIT_MOVE;
            break;
        }

        case MAZE_STATE_WAIT_MOVE:
        {
            if (solver->cfg.motion_done == 0)
            {
                FinishWithStatus(solver, MAZE_SOLVER_ERROR);
                break;
            }

            if (!solver->cfg.motion_done(solver->cfg.user))
                break;

            uint16_t moved_cells = solver->move_cell_count;

            if (moved_cells == 0u)
                moved_cells = 1u;

            if (!UpdateLogicalPositionCells(solver, moved_cells))
            {
                FinishWithStatus(solver, MAZE_SOLVER_ERROR);
                break;
            }

            if (solver->run_mode == MAZE_RUN_EXPLORE)
            {
                if (solver->backtracking)
                {
                    if (solver->stack_size > 0u)
                        solver->stack_size--;
                    else
                    {
                        FinishWithStatus(solver, MAZE_SOLVER_ERROR);
                        break;
                    }
                }

                solver->state = MAZE_STATE_BEGIN_CELL;
            }
            else if ((solver->run_mode == MAZE_RUN_RETURN_TO_START) ||
                     (solver->run_mode == MAZE_RUN_FAST))
            {
                if ((solver->route_index + moved_cells) < solver->route_length)
                    solver->route_index += moved_cells;
                else
                    solver->route_index = solver->route_length;

                solver->move_cell_count = 1u;
                solver->state = MAZE_STATE_ROUTE_START_STEP;
            }
            else
            {
                FinishWithStatus(solver, MAZE_SOLVER_ERROR);
            }

            break;
        }

        case MAZE_STATE_IDLE:
        case MAZE_STATE_FINISHED:
        case MAZE_STATE_ERROR:
        default:
        {
            break;
        }
    }

    return solver->status;
}

uint8_t MazeSolver_GetX(const MazeSolver *solver)
{
    return (solver != 0) ? solver->x : 0u;
}

uint8_t MazeSolver_GetY(const MazeSolver *solver)
{
    return (solver != 0) ? solver->y : 0u;
}

MazeDirection MazeSolver_GetDir(const MazeSolver *solver)
{
    return (solver != 0) ? solver->dir : MAZE_DIR_NORTH;
}

MazeSolverStatus MazeSolver_GetStatus(const MazeSolver *solver)
{
    return (solver != 0) ? solver->status : MAZE_SOLVER_ERROR;
}

MazeRunMode MazeSolver_GetRunMode(const MazeSolver *solver)
{
    return (solver != 0) ? solver->run_mode : MAZE_RUN_IDLE;
}

uint16_t MazeSolver_GetRouteLength(const MazeSolver *solver)
{
    return (solver != 0) ? solver->route_length : 0u;
}

uint16_t MazeSolver_GetRouteIndex(const MazeSolver *solver)
{
    return (solver != 0) ? solver->route_index : 0u;
}
