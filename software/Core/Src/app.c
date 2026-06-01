 /*
 * app.c
 *
 *  Created on: May 29, 2026.
 *      Author: david
 */

#include "app.h"
#include "main.h"

#include "params.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "motor.h"
#include "encoder.h"
#include "neopixel.h"
#include "imu.h"
#include "velocity_pi.h"
#include "button.h"
#include "robot_controller.h"
#include "ir_sensors.h"
#include "motion.h"

#include "maze_solver.h"

#define APP_PI_F 3.14159265358979323846f

// Defined in main.c because of MX
extern ADC_HandleTypeDef  hadc1;
extern ADC_HandleTypeDef  hadc2;
extern SPI_HandleTypeDef  hspi1;
extern UART_HandleTypeDef huart2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim16;

/* ------------------------------------------------------------------------- */
/* Application objects                                                       */
/* ------------------------------------------------------------------------- */

Motor motor_left =
{
    .htim_in1 = &htim1,
    .channel_in1 = TIM_CHANNEL_1,

    .htim_in2 = &htim1,
    .channel_in2 = TIM_CHANNEL_2,

    .direction = PARAM_MOTOR_DIR_LEFT
};

Motor motor_right =
{
    .htim_in1 = &htim1,
    .channel_in1 = TIM_CHANNEL_3,

    .htim_in2 = &htim1,
    .channel_in2 = TIM_CHANNEL_4,

    .direction = PARAM_MOTOR_DIR_RIGHT
};

Encoder enc_left;
Encoder enc_right;

NeoPixel neopixel;

IMU imu;
float yaw;

VelocityPI pi_left;
VelocityPI pi_right;
float dt;

Button btn1;
Button btn2;
DipSwitch dip_sw;

IR_DemuxPins_t ir_demux = {
    .a0 = {DMUX_A0_GPIO_Port, DMUX_A0_Pin},
    .a1 = {DMUX_A1_GPIO_Port, DMUX_A1_Pin},
    .a2 = {DMUX_A2_GPIO_Port, DMUX_A2_Pin},
    .en = {DMUX_EN_GPIO_Port, DMUX_EN_Pin},

    .en_active_high = true,
};

RobotController robot;

static MotionController motion;
static MotionCommand motion_cmd;

static MazeSolver maze_solver;
static volatile bool maze_solver_active = false;
static volatile bool front_emergency_abort = false;

typedef enum
{
    APP_MAZE_PHASE_IDLE = 0,
    APP_MAZE_PHASE_EXPLORING,
    APP_MAZE_PHASE_RETURNING,
    APP_MAZE_PHASE_WAIT_FAST_BUTTON,
    APP_MAZE_PHASE_FAST_RUN
} AppMazePhase;

static volatile AppMazePhase maze_phase = APP_MAZE_PHASE_IDLE;

/* ------------------------------------------------------------------------- */
/* Helpers                                                                   */
/* ------------------------------------------------------------------------- */

int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

static float Timer_GetUpdatePeriod_s(TIM_HandleTypeDef *htim)
{
    uint32_t timer_clk_hz;

    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();

    if ((RCC->CFGR & RCC_CFGR_PPRE1) == RCC_CFGR_PPRE1_DIV1)
        timer_clk_hz = pclk1;
    else
        timer_clk_hz = 2U * pclk1;

    uint32_t psc = htim->Instance->PSC;
    uint32_t arr = htim->Instance->ARR;

    float update_freq_hz =
        (float)timer_clk_hz / ((float)(psc + 1U) * (float)(arr + 1U));

    return 1.0f / update_freq_hz;
}

void _integrate_omega(float omega, float dt_s, float *yaw_out)
{
    *yaw_out += omega * dt_s;
}

static void App_GetMotionFeedback(MotionFeedback *fb)
{
    fb->x_m = RobotController_GetPoseX(&robot);
    fb->y_m = RobotController_GetPoseY(&robot);

    fb->yaw_rad = yaw;

    fb->linear_speed_mps = RobotController_GetLinearVelocity(&robot);
    fb->angular_speed_radps = IMU_GetGyroZRad(&imu);
}

static float App_ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value)
        return min_value;

    if (value > max_value)
        return max_value;

    return value;
}

typedef struct
{
    bool have_filter;
    bool center_valid;
    bool front_stop;
    bool front_abort;

    float left_diag;
    float right_diag;
    float front_l;
    float front_r;

    float center_omega_radps;
    uint16_t age_samples;

} AppWallAssist;

static AppWallAssist wall_assist;

static void App_ResetWallAssist(void)
{
    wall_assist.have_filter = false;
    wall_assist.center_valid = false;
    wall_assist.front_stop = false;
    wall_assist.front_abort = false;

    wall_assist.left_diag = 0.0f;
    wall_assist.right_diag = 0.0f;
    wall_assist.front_l = 0.0f;
    wall_assist.front_r = 0.0f;

    wall_assist.center_omega_radps = 0.0f;
    wall_assist.age_samples = PARAM_CENTERING_STALE_SAMPLES + 1u;
}

static void App_UpdateWallAssistFromIr(void)
{
    if (!PARAM_CENTERING_ENABLE && !PARAM_FRONT_AC_ENABLE)
        return;

    if (!IR_Sensors_FrameReady())
    {
        if (wall_assist.age_samples < 255u)
            wall_assist.age_samples++;

        return;
    }

    const int32_t *ir = IR_Sensors_GetSignal();

    float right_diag_raw = (float)ir[PARAM_IR_IDX_RIGHT_DIAG];
    float left_diag_raw  = (float)ir[PARAM_IR_IDX_LEFT_DIAG];
    float front_r_raw    = (float)ir[PARAM_IR_IDX_FRONT_R];
    float front_l_raw    = (float)ir[PARAM_IR_IDX_FRONT_L];

    if (!wall_assist.have_filter)
    {
        wall_assist.right_diag = right_diag_raw;
        wall_assist.left_diag = left_diag_raw;
        wall_assist.front_r = front_r_raw;
        wall_assist.front_l = front_l_raw;
        wall_assist.have_filter = true;
    }
    else
    {
        float a = PARAM_CENTERING_FILTER_ALPHA;

        if (a < 0.0f)
            a = 0.0f;
        else if (a > 1.0f)
            a = 1.0f;

        wall_assist.right_diag += a * (right_diag_raw - wall_assist.right_diag);
        wall_assist.left_diag  += a * (left_diag_raw  - wall_assist.left_diag);
        wall_assist.front_r    += a * (front_r_raw    - wall_assist.front_r);
        wall_assist.front_l    += a * (front_l_raw    - wall_assist.front_l);
    }

    bool right_wall = wall_assist.right_diag > (float)PARAM_CENTERING_DIAG_WALL_TH;
    bool left_wall  = wall_assist.left_diag  > (float)PARAM_CENTERING_DIAG_WALL_TH;

    float front_max_for_center = wall_assist.front_l;
    if (wall_assist.front_r > front_max_for_center)
        front_max_for_center = wall_assist.front_r;

    bool front_not_close =
        front_max_for_center < (float)PARAM_CENTERING_FRONT_DISABLE_TH;

    bool right_diag_not_corner =
        wall_assist.right_diag < (float)PARAM_CENTERING_DIAG_MAX_VALID_TH;

    bool left_diag_not_corner =
        wall_assist.left_diag < (float)PARAM_CENTERING_DIAG_MAX_VALID_TH;

    bool sensors_ok =
        front_not_close &&
        right_diag_not_corner &&
        left_diag_not_corner;

    float error_counts = 0.0f;

    /* Both walls: normal balancing. */
    if (sensors_ok && right_wall && left_wall)
    {
        error_counts = wall_assist.right_diag - wall_assist.left_diag;
        wall_assist.center_valid = true;
    }
    /* Only left wall: follow left wall using a target reading. */
    else if (sensors_ok && left_wall)
    {
        error_counts = PARAM_CENTERING_LEFT_DIAG_TARGET - wall_assist.left_diag;
        wall_assist.center_valid = true;
    }
    /* Only right wall: follow right wall using a target reading. */
    else if (sensors_ok && right_wall)
    {
        error_counts = wall_assist.right_diag - PARAM_CENTERING_RIGHT_DIAG_TARGET;
        wall_assist.center_valid = true;
    }
    else
    {
        error_counts = 0.0f;
        wall_assist.center_valid = false;
    }

    if (fabsf(error_counts) < (float)PARAM_CENTERING_DEADBAND_COUNTS)
        error_counts = 0.0f;

    wall_assist.center_omega_radps =
        PARAM_CENTERING_SIGN *
        PARAM_CENTERING_KP_RADPS_PER_COUNT *
        error_counts;

    wall_assist.center_omega_radps =
        App_ClampFloat(wall_assist.center_omega_radps,
                       -PARAM_CENTERING_MAX_OMEGA_RADPS,
                       PARAM_CENTERING_MAX_OMEGA_RADPS);

    float front_max = wall_assist.front_l;
    if (wall_assist.front_r > front_max)
        front_max = wall_assist.front_r;

    wall_assist.front_stop = front_max > (float)PARAM_FRONT_AC_STOP_TH;
    wall_assist.front_abort = front_max > (float)PARAM_FRONT_AC_ABORT_TH;

    wall_assist.age_samples = 0u;

    /* Consume this frame so TIM3 can start the next continuous IR scan. */
    IR_Sensors_ClearFrameReady();
}

static void App_ApplyWallAssist(MotionCommand *cmd)
{
    if ((cmd == NULL) || !cmd->active)
        return;

    if (Motion_GetMode(&motion) != MOTION_STRAIGHT)
        return;

    if (wall_assist.age_samples > PARAM_CENTERING_STALE_SAMPLES)
        return;

    float remaining_m = fabsf(Motion_GetRemaining(&motion));

    bool far_enough_from_cell_end =
        remaining_m > PARAM_CENTERING_DISABLE_END_REMAINING_M;

    if (PARAM_CENTERING_ENABLE &&
        wall_assist.center_valid &&
        far_enough_from_cell_end)
    {
        cmd->angular_radps += wall_assist.center_omega_radps;
        cmd->angular_radps =
            App_ClampFloat(cmd->angular_radps,
                           -PARAM_CENTERING_TOTAL_OMEGA_LIMIT_RADPS,
                           PARAM_CENTERING_TOTAL_OMEGA_LIMIT_RADPS);
    }
}

static bool App_HandleFrontAntiCollision(MotionCommand *cmd)
{
    if ((cmd == NULL) || !cmd->active)
        return false;

    if (!PARAM_FRONT_AC_ENABLE)
        return false;

    if (Motion_GetMode(&motion) != MOTION_STRAIGHT)
        return false;

    if (wall_assist.age_samples > PARAM_CENTERING_STALE_SAMPLES)
        return false;

    float remaining_m = fabsf(Motion_GetRemaining(&motion));

    if (wall_assist.front_abort &&
        (remaining_m > PARAM_FRONT_AC_FINISH_REMAINING_M))
    {
        Motion_Stop(&motion);
        front_emergency_abort = true;

        cmd->linear_mps = 0.0f;
        cmd->angular_radps = 0.0f;
        cmd->active = false;

        return true;
    }

    if (wall_assist.front_stop &&
        (remaining_m <= PARAM_FRONT_AC_FINISH_REMAINING_M))
    {
        Motion_ForceDone(&motion);

        cmd->linear_mps = 0.0f;
        cmd->angular_radps = 0.0f;
        cmd->active = false;

        return true;
    }

    return false;
}

/* ------------------------------------------------------------------------- */
/* HAL interrupt callbacks                                                   */
/* ------------------------------------------------------------------------- */

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM8)
        NeoPixel_DMA_Callback(&neopixel, htim);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == btn1.pin)
        Button_EXTI_Callback(&btn1, GPIO_Pin, &htim7);
    else if (GPIO_Pin == btn2.pin)
        Button_EXTI_Callback(&btn2, GPIO_Pin, &htim7);

    if (GPIO_Pin == IMU_INT2_Pin)
        IMU_NotifyDataReady(&imu);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
        if (!IR_Sensors_IsBusy() && !IR_Sensors_FrameReady())
            IR_Sensors_StartFrame();

    if (htim->Instance == TIM6)
    {
        _integrate_omega(IMU_GetGyroZRad(&imu), dt, &yaw);

        if (maze_solver_active && (Motion_GetMode(&motion) == MOTION_STRAIGHT))
            App_UpdateWallAssistFromIr();

        MotionFeedback fb;
        App_GetMotionFeedback(&fb);
        Motion_Update(&motion, &fb, &motion_cmd);

        if (maze_solver_active && (Motion_GetMode(&motion) == MOTION_STRAIGHT))
        {
            App_ApplyWallAssist(&motion_cmd);
            App_HandleFrontAntiCollision(&motion_cmd);
        }

        if (motion_cmd.active)
            RobotController_SetCmdVel(&robot, motion_cmd.linear_mps, motion_cmd.angular_radps);
        else
            RobotController_Stop(&robot);

        RobotController_Update(&robot, dt);
    }

    if (htim->Instance == TIM7)
        Button_TIM_PeriodElapsedCallback(htim);

    if (htim->Instance == TIM16)
        IR_Sensors_OnTimerElapsed(htim);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    IR_Sensors_OnAdcConvCplt(hadc);
}

static void App_MazeBeginWallRead(void *user)
{
    (void)user;

    /*
     * Important: discard old IR frame, then force a fresh frame.
     * TIM3 also starts frames automatically, but this reduces waiting.
     */
    IR_Sensors_ClearFrameReady();

    if (!IR_Sensors_IsBusy())
        IR_Sensors_StartFrame();
}

static bool App_MazeReadWalls(MazeWallReading *walls, void *user)
{
    (void)user;

    if (walls == NULL)
        return false;

    if (!IR_Sensors_FrameReady())
        return false;

    const int32_t *ir = IR_Sensors_GetSignal();

    /*
     * Physical mapping is defined in params.h.
     * If the robot behaves mirrored, swap PARAM_IR_IDX_LEFT_* and
     * PARAM_IR_IDX_RIGHT_* in params.h or swap left/right here.
     */
    int32_t ir_right_h    = ir[PARAM_IR_IDX_RIGHT_H];
    int32_t ir_right_diag = ir[PARAM_IR_IDX_RIGHT_DIAG];
    int32_t ir_front_r    = ir[PARAM_IR_IDX_FRONT_R];
    int32_t ir_front_l    = ir[PARAM_IR_IDX_FRONT_L];
    int32_t ir_left_diag  = ir[PARAM_IR_IDX_LEFT_DIAG];
    int32_t ir_left_h     = ir[PARAM_IR_IDX_LEFT_H];

    walls->left =
        (ir_left_h    > PARAM_MAZE_LEFT_WALL_TH);

    walls->front =
        (ir_front_l > PARAM_MAZE_FRONT_WALL_TH) ||
        (ir_front_r > PARAM_MAZE_FRONT_WALL_TH);

    walls->right =
        (ir_right_h    > PARAM_MAZE_RIGHT_WALL_TH);

    printf("Cell %u,%u dir=%d | RH=%ld RD=%ld FR=%ld FL=%ld LD=%ld LH=%ld | L%d F%d R%d\r\n",
           MazeSolver_GetX(&maze_solver),
           MazeSolver_GetY(&maze_solver),
           MazeSolver_GetDir(&maze_solver),
           (long)ir_right_h,
           (long)ir_right_diag,
           (long)ir_front_r,
           (long)ir_front_l,
           (long)ir_left_diag,
           (long)ir_left_h,
           walls->left ? 1 : 0,
           walls->front ? 1 : 0,
           walls->right ? 1 : 0);

    IR_Sensors_ClearFrameReady();

    return true;
}

static bool App_MazeMotionDone(void *user)
{
    (void)user;

    if (Motion_IsDone(&motion))
    {
        Motion_ClearDone(&motion);
        return true;
    }

    return false;
}

static bool App_MazeStartMoveCells(uint16_t cells, void *user)
{
    (void)user;

    if (cells == 0u)
        cells = 1u;

    if (Motion_IsBusy(&motion))
        return false;

    if (Motion_IsDone(&motion))
        Motion_ClearDone(&motion);

    App_ResetWallAssist();
    IR_Sensors_ClearFrameReady();

    MotionFeedback fb;
    App_GetMotionFeedback(&fb);

    float cruise_speed = PARAM_MAZE_CRUISE_SPEED_MPS;

    if (maze_phase == APP_MAZE_PHASE_RETURNING)
        cruise_speed = PARAM_MAZE_RETURN_CRUISE_SPEED_MPS;
    else if (maze_phase == APP_MAZE_PHASE_FAST_RUN)
        cruise_speed = PARAM_MAZE_FAST_CRUISE_SPEED_MPS;

    float distance_m = PARAM_MAZE_CELL_DISTANCE_M * (float)cells;

    bool ok = Motion_StartStraight(&motion,
                                   &fb,
                                   distance_m,
                                   cruise_speed);

    if (ok)
        printf("Move %u cell(s), distance=%.3f m, speed=%.2f\r\n",
               (unsigned int)cells,
               distance_m,
               cruise_speed);

    return ok;
}

static bool App_MazeStartMoveCell(void *user)
{
    return App_MazeStartMoveCells(1u, user);
}

static bool App_MazeStartTurnDeg(int16_t degrees, void *user)
{
    (void)user;

    if (Motion_IsBusy(&motion))
        return false;

    if (Motion_IsDone(&motion))
        Motion_ClearDone(&motion);

    App_ResetWallAssist();

    MotionFeedback fb;
    App_GetMotionFeedback(&fb);

    float turn_speed = PARAM_MAZE_TURN_SPEED_RADPS;

    if (maze_phase == APP_MAZE_PHASE_RETURNING)
        turn_speed = PARAM_MAZE_RETURN_TURN_SPEED_RADPS;
    else if (maze_phase == APP_MAZE_PHASE_FAST_RUN)
        turn_speed = PARAM_MAZE_FAST_TURN_SPEED_RADPS;

    float angle_rad = ((float)degrees) * APP_PI_F / 180.0f;

    bool ok = Motion_StartTurn(&motion,
                               &fb,
                               angle_rad,
                               turn_speed);

    if (ok)
        printf("Turn %d deg speed=%.2f\r\n", degrees, turn_speed);

    return ok;
}

static void App_MazeStopMotion(void *user)
{
    (void)user;

    Motion_Stop(&motion);
    Motion_ClearDone(&motion);

    RobotController_Stop(&robot);
    App_ResetWallAssist();
}

static void App_MazeLog(const char *msg, void *user)
{
    (void)user;

    if (msg != NULL)
        printf("%s\r\n", msg);
}

static void App_StartMazeSolver(void)
{
    printf("Starting maze solver\r\n");

    maze_solver_active = false;
    maze_phase = APP_MAZE_PHASE_IDLE;
    front_emergency_abort = false;

    App_MazeStopMotion(NULL);

    RobotController_ResetOdometry(&robot);
    yaw = 0.0f;

    IR_Sensors_ClearFrameReady();

    MazeSolverConfig cfg = {
        .begin_wall_read = App_MazeBeginWallRead,
        .read_walls = App_MazeReadWalls,

        .start_move_cell = App_MazeStartMoveCell,
        .start_move_cells = App_MazeStartMoveCells,
        .start_turn_deg = App_MazeStartTurnDeg,
        .motion_done = App_MazeMotionDone,
        .stop_motion = App_MazeStopMotion,

        .log = App_MazeLog,
        .user = NULL,

        .start_x = 0,
        .start_y = 0,
        .start_dir = MAZE_DIR_NORTH,

        .left_bias = true,
        .fast_run_join_straights = false
    };

    MazeSolver_Init(&maze_solver, &cfg);
    MazeSolver_Start(&maze_solver);

    maze_phase = APP_MAZE_PHASE_EXPLORING;
    maze_solver_active = true;

    NeoPixel_SetColor(&neopixel, COLOR_BLUE);
    NeoPixel_Show(&neopixel);
}

static void App_StopMazeSolver(void)
{
    printf("Stopping maze solver\r\n");

    MazeSolver_Stop(&maze_solver);
    maze_solver_active = false;
    maze_phase = APP_MAZE_PHASE_IDLE;

    App_MazeStopMotion(NULL);

    NeoPixel_SetColor(&neopixel, COLOR_RED);
    NeoPixel_Show(&neopixel);
}

static void App_StartFastRunAfterButton(void)
{
    App_MazeStopMotion(NULL);

    /* After tire cleaning, place the mouse back in the normal start pose:
     * cell (0,0), facing NORTH.
     */
    maze_solver.x = maze_solver.cfg.start_x;
    maze_solver.y = maze_solver.cfg.start_y;
    maze_solver.dir = maze_solver.cfg.start_dir;
    maze_solver.target_dir = maze_solver.cfg.start_dir;

    RobotController_ResetOdometry(&robot);
    yaw = 0.0f;

    bool join_straights = DipSwitch_IsOn(&dip_sw, 1);
    maze_solver.cfg.fast_run_join_straights = join_straights;

    printf("DIP1 straight-join fast run: %s\r\n",
           join_straights ? "ON" : "OFF");

    if (MazeSolver_StartFastRun(&maze_solver))
    {
        maze_phase = APP_MAZE_PHASE_FAST_RUN;
        maze_solver_active = true;

        printf("Fast route length: %u cells\r\n",
               MazeSolver_GetRouteLength(&maze_solver));

        NeoPixel_SetColor(&neopixel, COLOR_MAGENTA);
        NeoPixel_Show(&neopixel);
        return;
    }

    printf("Could not plan fast route\r\n");

    maze_solver_active = false;
    maze_phase = APP_MAZE_PHASE_IDLE;

    App_MazeStopMotion(NULL);

    NeoPixel_SetColor(&neopixel, COLOR_RED);
    NeoPixel_Show(&neopixel);
}

static void App_HandleMazeSolverFinished(MazeSolverStatus status)
{
    printf("Maze solver finished. status=%d phase=%d x=%u y=%u dir=%d steps=%lu route=%u/%u\r\n",
           status,
           maze_phase,
           MazeSolver_GetX(&maze_solver),
           MazeSolver_GetY(&maze_solver),
           MazeSolver_GetDir(&maze_solver),
           (unsigned long)maze_solver.steps,
           MazeSolver_GetRouteIndex(&maze_solver),
           MazeSolver_GetRouteLength(&maze_solver));

    if ((status == MAZE_SOLVER_REACHED_GOAL) &&
        (maze_phase == APP_MAZE_PHASE_EXPLORING))
    {
        App_MazeStopMotion(NULL);

        if (MazeSolver_StartReturnToStart(&maze_solver))
        {
            maze_phase = APP_MAZE_PHASE_RETURNING;
            maze_solver_active = true;

            printf("Return route length: %u cells\r\n",
                   MazeSolver_GetRouteLength(&maze_solver));

            NeoPixel_SetColor(&neopixel, COLOR_ORANGE);
            NeoPixel_Show(&neopixel);
            return;
        }

        printf("Could not plan return route\r\n");
        status = MAZE_SOLVER_ERROR;
    }

    if ((status == MAZE_SOLVER_RETURNED_TO_START) &&
        (maze_phase == APP_MAZE_PHASE_RETURNING))
    {
        App_MazeStopMotion(NULL);

        maze_solver_active = false;
        maze_phase = APP_MAZE_PHASE_WAIT_FAST_BUTTON;

        printf("Returned to start. Clean tires, place mouse at start facing NORTH, then press BTN1 for fast run.\r\n");
        printf("DIP1 ON = fast run joins straight cells. DIP1 OFF = old cell-by-cell fast run.\r\n");

        NeoPixel_SetColor(&neopixel, COLOR_ORANGE);
        NeoPixel_Show(&neopixel);

        return;
    }

    maze_solver_active = false;
    maze_phase = APP_MAZE_PHASE_IDLE;
    App_MazeStopMotion(NULL);

    if (status == MAZE_SOLVER_FAST_RUN_DONE)
        NeoPixel_SetColor(&neopixel, COLOR_GREEN);
    else if (status == MAZE_SOLVER_REACHED_GOAL)
        NeoPixel_SetColor(&neopixel, COLOR_GREEN);
    else
        NeoPixel_SetColor(&neopixel, COLOR_RED);

    NeoPixel_Show(&neopixel);
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */

void App_Init(void)
{
    dt = Timer_GetUpdatePeriod_s(&htim6);

    Motor_Init(&motor_left);
    Motor_Init(&motor_right);

    Encoder_Init(&enc_left,  &htim2, PARAM_ENC_TICKS_PER_REV, PARAM_ENC_DIR_LEFT, PARAM_ENC_VEL_FILTER_TAU_S);
    Encoder_Init(&enc_right, &htim4, PARAM_ENC_TICKS_PER_REV,  PARAM_ENC_DIR_RIGHT, PARAM_ENC_VEL_FILTER_TAU_S);

    VelocityPI_Init(&pi_left,
                    PARAM_PI_KP,
					PARAM_PI_KI,
                    dt,
                    -PARAM_PI_MAX_PWM_OUT,
					PARAM_PI_MAX_PWM_OUT);

    VelocityPI_Init(&pi_right,
    				PARAM_PI_KP,
					PARAM_PI_KI,
                    dt,
                    -PARAM_PI_MAX_PWM_OUT,
					PARAM_PI_MAX_PWM_OUT);

    VelocityPI_SetSetpoint(&pi_left,  0.0f);
    VelocityPI_SetSetpoint(&pi_right, 0.0f);

    NeoPixel_Init(&neopixel, &htim8, TIM_CHANNEL_1);
    NeoPixel_Off(&neopixel);

    Button_Init(&btn1, BTN_1_GPIO_Port, BTN_1_Pin);
    Button_Init(&btn2, BTN_2_GPIO_Port, BTN_2_Pin);

    DipSwitch_Init(&dip_sw,
                   DIP_1_GPIO_Port, DIP_1_Pin,
                   DIP_2_GPIO_Port, DIP_2_Pin,
                   DIP_3_GPIO_Port, DIP_3_Pin,
                   GPIO_PIN_SET);

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    IR_Sensors_Init(&hadc1, &hadc2, &htim16, &ir_demux);

    RobotController_Init(&robot,
                         &motor_left, &motor_right,
                         &enc_left,   &enc_right,
                         &pi_left,    &pi_right);

    IMU_Status imu_status;

    imu_status = IMU_Init(
            &imu,
            &hspi1,
            IMU_CS_GPIO_Port,
            IMU_CS_Pin,
            IMU_GYRO_ODR_1_66_KHZ,
            IMU_LPF1_ENABLED);

    if (imu_status == IMU_OK)
        NeoPixel_SetColor(&neopixel, COLOR_GREEN);
    else
        NeoPixel_SetColor(&neopixel, COLOR_RED);

    NeoPixel_Show(&neopixel);

    /* IMU calibration */
    HAL_Delay(PARAM_IMU_CALIB_SETTLE_MS);

    NeoPixel_SetColor(&neopixel, COLOR_BLUE);
    NeoPixel_Show(&neopixel);

    if (imu_status == IMU_OK)
        IMU_CalibrateGyro(&imu, PARAM_IMU_CALIB_SAMPLES);

    NeoPixel_Off(&neopixel);

    IMU_Update(&imu);

    MotionConfig cfg;

    Motion_DefaultConfig(&cfg);

    cfg.control_dt_s = dt;   /* 100 Hz TIM6 loop */

    cfg.max_linear_speed_mps = PARAM_MOTION_MAX_LINEAR_SPEED_MPS;
    cfg.max_linear_accel_mps2 = PARAM_MOTION_MAX_LINEAR_ACCEL_MPS2;

    cfg.max_angular_speed_radps = PARAM_MOTION_MAX_ANGULAR_SPEED_RADPS;
    cfg.max_angular_accel_radps2 = PARAM_MOTION_MAX_ANGULAR_ACCEL_RADPS2;

    cfg.distance_kp = PARAM_MOTION_DISTANCE_KP;
    cfg.heading_kp = PARAM_MOTION_HEADING_KP;
    cfg.turn_kp = PARAM_MOTION_TURN_KP;

    cfg.use_lateral_odometry_correction = PARAM_MOTION_USE_LATERAL_CORR;
    cfg.lateral_kp = PARAM_MOTION_LATERAL_KP;

    Motion_Init(&motion, &cfg);
    App_ResetWallAssist();

    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_Base_Start_IT(&htim6);
}

uint32_t lastms = 0;

NeoPixel_Color colors[7] = {
		COLOR_RED,
		COLOR_GREEN,
		COLOR_BLUE,
		COLOR_YELLOW,
		COLOR_ORANGE,
		COLOR_MAGENTA,
		COLOR_OFF
};
uint8_t color_idx = 0;

void App_Loop(void)
{
	if (IR_Sensors_FrameReady())
	{
		uint32_t now = HAL_GetTick();
		if (now - lastms >= 50)
			lastms = now;
		IR_Sensors_ClearFrameReady();
	}

	IMU_UpdateIfReady(&imu);

	NeoPixel_SetColor(&neopixel, colors[color_idx]);
	NeoPixel_Show(&neopixel);
	color_idx = (color_idx + 1) % 7;

	/* ---- Telemetry to ESP32 dashboard (USART2), ~40 Hz ---- */
	static uint32_t tlm_last_ms = 0;
	uint32_t tlm_now = HAL_GetTick();
	if (tlm_now - tlm_last_ms >= 50)
	{
		tlm_last_ms = tlm_now;

		const int32_t *ir = IR_Sensors_GetSignal();
		NeoPixel_Color color = NeoPixel_GetColor(&neopixel);

		bool b1 = (HAL_GPIO_ReadPin(BTN_1_GPIO_Port, BTN_1_Pin) == GPIO_PIN_RESET);
		bool b2 = (HAL_GPIO_ReadPin(BTN_2_GPIO_Port, BTN_2_Pin) == GPIO_PIN_RESET);

		printf("{\"ir\":[%ld,%ld,%ld,%ld,%ld,%ld],"
			   "\"gyro\":{\"z\":%.2f},\"yaw\":%.3f,"
			   "\"odo\":{\"l\":%ld,\"r\":%ld},"
			   "\"speed\":{\"l\":%.2f,\"r\":%.2f},"
			   "\"pose\":{\"x\":%.3f,\"y\":%.3f,\"th\":%.3f},"
			   "\"v\":%.3f,\"w\":%.3f,"
			   "\"btn\":{\"b1\":%d,\"b2\":%d},"
			   "\"dip\":%d,"
			   "\"led\":{\"r\":%d,\"g\":%d,\"b\":%d}}\r\n",
			   (long)ir[0], (long)ir[1], (long)ir[2],
			   (long)ir[3], (long)ir[4], (long)ir[5],
			   IMU_GetGyroZDeg(&imu), yaw,
			   (long)Encoder_GetPositionTicks(&enc_left),
			   (long)Encoder_GetPositionTicks(&enc_right),
			   Encoder_GetVelocityRadPerSecond(&enc_left),
			   Encoder_GetVelocityRadPerSecond(&enc_right),
			   RobotController_GetPoseX(&robot),
			   RobotController_GetPoseY(&robot),
			   RobotController_GetPoseTheta(&robot),
			   RobotController_GetLinearVelocity(&robot),
			   RobotController_GetAngularVelocity(&robot),
			   b1 ? 0 : 1, b2 ? 0 : 1,
			   DipSwitch_GetValue(&dip_sw),
			   color.red, color.green, color.blue);
	}
}
