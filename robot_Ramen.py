#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# =============================================================================
# [실제 로봇(Real Robot) 실행 명령어]
# $ ros2 launch dsr_bringup2 dsr_bringup2.launch.py model:=e0509 host:=110.120.1.6 mode:=real
# =============================================================================

import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import DrlStart
import DR_init
import time

# [수정] 작동 확인된 그리퍼 컨트롤러 임포트
from dsr_example.simple.gripper_drl_controller import GripperController

# [필수] 원호 이동을 위한 메시지 타입 임포트
from dsr_msgs2.srv import MoveCircle
from std_msgs.msg import Float64MultiArray

# --- [신라면 설정] ---
SHIN_RAMYUN_TIME = 270 # 4분 30초

# --- [커스텀 원호 이동 함수] ---
def custom_movec(node, pos1, pos2, vel, acc, mode=0):
    if not hasattr(node, 'cli_move_circle'):
        node.cli_move_circle = node.create_client(MoveCircle, '/dsr01/motion/move_circle')
    
    cli = node.cli_move_circle
    if not cli.service_is_ready():
        cli.wait_for_service(timeout_sec=1.0)
    
    req = MoveCircle.Request()
    msg1 = Float64MultiArray()
    msg1.data = [float(x) for x in pos1]
    msg2 = Float64MultiArray()
    msg2.data = [float(x) for x in pos2]
    
    req.pos = [msg1, msg2]
    req.vel = [float(vel), 0.0]
    req.acc = [float(acc), 0.0]
    req.time = 0.0
    req.radius = 0.0
    req.mode = mode
    req.blend_type = 0
    req.sync_type = 0
    
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    return future.result()

# --- [안전한 그리퍼 동작 함수] ---
def secure_gripper_move(node, gripper, val, delay=1.4):
    status = "OPEN" if val < 600 else "CLOSE"
    node.get_logger().info(f"🔒 그리퍼 동작: {status} ({val})")
    gripper.move(val)
    time.sleep(delay) 
    node.get_logger().info(f"   -> 동작 완료 ({delay}s)")

# --- [댄스 1: 좌우/위아래 흔들기] ---
def wait_and_dance(node, start_time, target_elapsed_sec, movej, p_dance_1, p_dance_2, vel, acc, dance_name="댄스"):
    node.get_logger().info(f"💃 {dance_name} 시작! (맛있어져라~)")
    while True:
        current_elapsed = time.time() - start_time
        remaining = target_elapsed_sec - current_elapsed
        
        if remaining <= 0:
            break
            
        node.get_logger().info(f"⏳ 끓이는 중... ({int(remaining)}초 남음)")
        
        if remaining > 5.0:
            movej(p_dance_1, vel * 0.5, acc * 0.5)
            movej(p_dance_2, vel * 0.5, acc * 0.5)
        else:
            time.sleep(min(1.0, remaining))

# --- [댄스 2: J6 손목 빙빙 돌리기] ---
def wait_and_rotate_J6(node, start_time, target_elapsed_sec, movej, p_base, vel, acc):
    node.get_logger().info("🌪️ 회오리 댄스 (J6 회전) 시작!")
    
    # J6축을 기준으로 +90도, -90도 회전하는 좌표 생성
    p_rot_cw = list(p_base)
    p_rot_ccw = list(p_base)
    
    # J6 범위 체크 및 회전 (단순히 값을 더하고 뺌)
    p_rot_cw[5] += 90.0
    p_rot_ccw[5] -= 90.0
    
    while True:
        current_elapsed = time.time() - start_time
        remaining = target_elapsed_sec - current_elapsed
        
        if remaining <= 0:
            break
            
        node.get_logger().info(f"⏳ 끓이는 중... ({int(remaining)}초 남음)")
        
        if remaining > 5.0:
            # 시계 방향 회전
            movej(p_rot_cw, vel * 0.8, acc * 0.5)
            # 반시계 방향 회전
            movej(p_rot_ccw, vel * 0.8, acc * 0.5)
        else:
            time.sleep(min(1.0, remaining))
            
    # 대기 후 정자세 복귀
    movej(p_base, vel, acc)

def wait_cooking_time(node, start_time, target_elapsed_sec):
    while True:
        current_elapsed = time.time() - start_time
        remaining = target_elapsed_sec - current_elapsed
        if remaining <= 0: break
        node.get_logger().info(f"⏳ 끓이는 중... ({int(remaining)}초 남음)")
        time.sleep(min(10.0, remaining))

def main(args=None):
    rclpy.init(args=args)
    
    ROBOT_ID = "dsr01"
    ROBOT_MODEL = "e0509"
    VEL = 60.0
    ACC = 30.0
    
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    node = rclpy.create_node('ramen_chef_drl', namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    import DSR_ROBOT2 as drl
    from DSR_ROBOT2 import set_robot_mode, ROBOT_MODE_AUTONOMOUS, posj, posx, movej, wait

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    wait(1.0)

    gripper = None
    try:
        gripper = GripperController(node=node, namespace=ROBOT_ID)
        if not gripper.initialize():
            node.get_logger().error("Gripper initialization failed.")
            return

        node.get_logger().info("Gripper Wake-up...")
        wait(2.9)
        
        GRIPPER_OPEN = 0
        GRIPPER_CLOSE = 600 
        GRIPPER_POT_CLOSE = 1000 # [추가] 냄비 파지용 강한 그립력

        # 워밍업
        gripper.move(GRIPPER_CLOSE)
        wait(1.4)
        gripper.move(GRIPPER_OPEN)
        wait(1.4)

        # --- [좌표 정의] ---
        p_home = posj(0.0, 0.0, 90.0, 0.0, 90.0, -6.81)
        
        # 면 잡기 (대기 -> 진입 -> 들어올리기)
        p_noodle_standby  = posj(-20.36, 33.45, 128.57, -12.55, -65.39, -3.43)
        p_noodle_grip     = posj(-20.73, 40.49, 118.7, -14.3, -65.39, -6.81)
        p_noodle_lift     = posj(-16.7, 29.3, 129.91, -5.02, -69.73, -6.81)
        
        # 냄비 조리 위치
        p_pot_approach    = posj(9.48, 29.3, 133.09, 11.16, -65.28, -6.81)
        p_pot_place       = posj(8.74, 23.29, 120.96, 11.16, -25.94, -6.81)
        p_pot_standby     = posj(0, -30, 120, 0, 0, -6.81)
        
        # 댄스 좌표
        p_dance_swing_1 = posj(15, -30, 120, 0, 0, 20)
        p_dance_swing_2 = posj(-15, -30, 120, 0, 0, -30)
        p_dance_nod_1 = posj(0, -20, 110, 0, 20, -6.81) 
        p_dance_nod_2 = posj(0, -40, 130, 0, -20, -6.81)
        
        # 휘젓기 & 괴면 (면 잡는 공통 위치)
        p_stir_prep       = posj(4.53, 26.28, 101.55, 3.63, 21.98, 90)
        c1_via            = posx(50, 30, 0, 0, 0, 0)
        c1_target         = posx(0, -30, 0, 0, 0, 0)
        c2_via            = posx(0, 30, 0, 0, 0, 0)
        c2_target         = posx(0, -30, 0, 0, 0, 0)
        
        # [수정] 들었다 놨다 할 때 X, Y축이 안 변하도록 베이스(J1~J4) 고정!
        # 기존 로직처럼 손목(J5)만 위로 20도 가량 꺾어서 들어 올리기 (21.98 -> 1.98)
        p_aeration_lift   = posj(4.53, 26.28, 101.55, 3.63, 1.98, 90)
        
        # 냄비 잡기 & 배달
        p_pot_grasp_prep  = posj(6.05, 36.59, 72.7, 1.65, 72.77, 0)   # 냄비 잡기 대기
        p_pot_grasp_real  = posj(5.85, 38.13, 75.47, 1.46, 66.94, 0)  # 냄비 잡기
        p_pot_lift_up     = posj(6.05, 36.59, 72.7, 1.65, 72.77, 0)   # 냄비 들어올리기
        p_delivery_left   = posj(19.67, 48.94, 48.91, 1.64, 78.15, 0) # 배달 위치 대기 & 홈 복 복귀 전 대기
        
        # [수정] Z축으로 약 1cm 정도 높게 내려놓기 위해 J2, J3, J5 관절 각도 미세 조정 
        # 기존: posj(19.67, 49.68, 56.53, 1.64, 74.68, 0)
        p_delivery_place  = posj(19.67, 49.40, 54.50, 1.64, 75.50, 0) # 냄비 배달 내려두기 (Z축 상향)

        # --- [시나리오 실행] ---
        print("\n" + "="*50)
        print("🍜 [START] 두산 로봇 자율 라면 조리 시스템 (신라면)")
        print("="*50)
        time.sleep(0.9)
        
        print("\n▶ [Phase 1] 면 파지 및 투하 (Pick & Drop)")
        print("   1-1. 홈 위치 초기화")
        movej(p_home, VEL, ACC)
        time.sleep(0.9)
        secure_gripper_move(node, gripper, GRIPPER_OPEN, delay=1.4)
        
        print("   1-2. 라면 사리 상공 접근 (대기)")
        movej(p_noodle_standby, VEL, ACC)
        time.sleep(0.9)
        
        print("   1-3. 파지 위치 진입 및 면 잡기")
        movej(p_noodle_grip, VEL, ACC)
        time.sleep(0.9)
        secure_gripper_move(node, gripper, GRIPPER_CLOSE, delay=1.4)
        
        print("   1-4. 면 들어 올리기 (수직 상승)")
        movej(p_noodle_lift, VEL, ACC)
        time.sleep(0.9)
        
        print("   1-5. 냄비 상공으로 이동")
        movej(p_pot_approach, VEL, ACC)
        time.sleep(1.4)
        
        print("   1-6. 냄비 내부로 면 투하")
        movej(p_pot_place, VEL, ACC)
        time.sleep(0.9)
        secure_gripper_move(node, gripper, GRIPPER_OPEN, delay=1.4)
        
        print("   1-7. 안전 높이로 복귀")
        movej(p_pot_approach, VEL, ACC)
        time.sleep(0.9)
        
        # --- 조리 타이머 시작 ---
        cooking_start_time = time.time()
        node.get_logger().info(f"🔥 조리 타이머 시작! (목표: {SHIN_RAMYUN_TIME}초)")

        print("\n▶ [Phase 2] 초기 끓이기 및 댄스 (Boiling & Idle UX)")
        movej(p_pot_standby, VEL, ACC)
        secure_gripper_move(node, gripper, GRIPPER_CLOSE, delay=1.4)
        wait_and_dance(node, cooking_start_time, 60, movej, p_dance_swing_1, p_dance_swing_2, VEL, ACC, "좌우 흔들기")
        movej(p_pot_standby, VEL, ACC) 

        print("\n▶ [Phase 3] 면 휘젓기 (Stirring - MoveC 궤적 제어)")
        secure_gripper_move(node, gripper, GRIPPER_OPEN, delay=1.4)
        
        print("   3-1. 휘젓기 준비 위치 진입")
        movej(p_stir_prep, VEL, ACC)
        time.sleep(0.9)
        
        print("   3-2. 면 파지 (Grasp)")
        secure_gripper_move(node, gripper, GRIPPER_CLOSE, delay=3.0)
        
        print("   3-3. 1차 원호 궤적 (MoveC)")
        custom_movec(node, c1_via, c1_target, VEL, ACC, mode=1)
        movej(p_stir_prep, VEL, ACC)
        time.sleep(0.4)
        
        print("   3-4. 2차 원호 궤적 (MoveC)")
        custom_movec(node, c2_via, c2_target, VEL, ACC, mode=1)
        time.sleep(0.9)
        
        print("   3-5. 면 놓기 (Release)")
        secure_gripper_move(node, gripper, GRIPPER_OPEN, delay=1.4)
        
        movej(p_pot_standby, VEL, ACC)
        print("   (대기 중 J6축 회전 댄스)")
        wait_and_rotate_J6(node, cooking_start_time, 120, movej, p_pot_standby, VEL, ACC)

        print("\n▶ [Phase 4] 1차 면 괴기 (1st Aeration - 관절 고정 정밀 제어)")
        print("   4-1. 파지 위치 진입")
        movej(p_stir_prep, VEL, ACC)
        time.sleep(0.9)
        
        print("   4-2. 면 잡기 (Grasp)")
        secure_gripper_move(node, gripper, GRIPPER_CLOSE, delay=3.0) 
        
        print("   4-3. 수직 상승 (Lift)")
        movej(p_aeration_lift, VEL, ACC)
        time.sleep(2.0)
        
        print("   4-4. 하강 (Down)")
        movej(p_stir_prep, VEL, ACC)  
        time.sleep(0.9)
        
        print("   4-5. 면 놓기 (Release)")
        secure_gripper_move(node, gripper, GRIPPER_OPEN, delay=1.4)
        
        movej(p_pot_standby, VEL, ACC)
        wait_and_dance(node, cooking_start_time, 180, movej, p_dance_swing_1, p_dance_swing_2, VEL, ACC, "좌우 흔들기")
        movej(p_pot_standby, VEL, ACC)
        
        print("\n▶ [Phase 5] 2차 면 휘젓기 (2nd Stirring)")
        print("   5-1. 휘젓기 준비 위치 진입")
        movej(p_stir_prep, VEL, ACC)
        time.sleep(0.9)
        
        print("   5-2. 면 파지 (Grasp)")
        secure_gripper_move(node, gripper, GRIPPER_CLOSE, delay=3.0)
        
        print("   5-3. 1차 원호 궤적 (MoveC)")
        custom_movec(node, c1_via, c1_target, VEL, ACC, mode=1)
        movej(p_stir_prep, VEL, ACC)
        time.sleep(0.4)
        
        print("   5-4. 2차 원호 궤적 (MoveC)")
        custom_movec(node, c2_via, c2_target, VEL, ACC, mode=1)
        time.sleep(0.9)
        
        print("   5-5. 면 놓기 (Release)")
        secure_gripper_move(node, gripper, GRIPPER_OPEN, delay=1.4)
        
        movej(p_pot_standby, VEL, ACC)
        time.sleep(1.0) # 짧은 대기 후 바로 2차 괴기로 진입
        
        print("\n▶ [Phase 6] 2차 면 괴기 (2nd Aeration)")
        print("   6-1. 파지 위치 진입")
        movej(p_stir_prep, VEL, ACC)  
        time.sleep(2.5)
        
        print("   6-2. 면 잡기 (Grasp)")
        secure_gripper_move(node, gripper, GRIPPER_CLOSE, delay=3.0)
        
        print("   6-3. 수직 상승 (Lift)")
        movej(p_aeration_lift, VEL, ACC)
        time.sleep(2.0)
        
        print("   6-4. 하강 (Down)")
        movej(p_stir_prep, VEL, ACC)  
        time.sleep(0.9)
        
        print("   6-5. 면 놓기 (Release)")
        secure_gripper_move(node, gripper, GRIPPER_OPEN, delay=1.4)
        
        print("\n▶ [Phase 7] 마무리 끓이기 (Final Boiling)")
        movej(p_pot_standby, VEL, ACC)
        wait_and_dance(node, cooking_start_time, SHIN_RAMYUN_TIME, movej, p_dance_nod_1, p_dance_nod_2, VEL, ACC, "끄덕이기 춤")
        movej(p_pot_standby, VEL, ACC)

        print("\n▶ [Phase 8] 조리 완료 및 냄비 서빙 (Serving - Grip Force Max)")
        secure_gripper_move(node, gripper, GRIPPER_OPEN, delay=2.0) 
        
        print("   8-1. 서빙 파지 준비 위치")
        movej(p_pot_grasp_prep, VEL, ACC)
        time.sleep(1.5)
        
        print("   8-2. 냄비 파지점 진입")
        movej(p_pot_grasp_real, VEL, ACC)
        time.sleep(2.0)
        
        print(f"   8-3. 최대 파지력({GRIPPER_POT_CLOSE})으로 냄비 잡기")
        secure_gripper_move(node, gripper, GRIPPER_POT_CLOSE, delay=4.0)
        
        print("   8-4. 수평 유지하며 들어 올리기")
        movej(p_pot_lift_up, VEL, ACC)
        time.sleep(2.0) 
        
        print("   8-5. 배달 위치로 이송")
        movej(p_delivery_left, VEL, ACC)
        time.sleep(3.0) 
        
        print("   8-6. 테이블에 내려놓기")
        movej(p_delivery_place, VEL, ACC)
        time.sleep(1.5)
        
        print("   8-7. 냄비 놓기 (Release)")
        secure_gripper_move(node, gripper, GRIPPER_OPEN, delay=2.0)
        
        print("   8-8. 홈 위치로 복귀")
        movej(p_delivery_left, VEL, ACC)
        time.sleep(1.0)
        movej(p_home, VEL, ACC)
        
        print("\n" + "="*50)
        print("🎉 모든 조리 및 서빙 프로세스 완료! 맛있게 드세요! 🍜")
        print("="*50 + "\n")

    except Exception as e:
        node.get_logger().error(f"An error occurred: {e}")
    finally:
        if gripper:
            gripper.terminate()
        rclpy.shutdown()
        node.get_logger().info("Shutdown complete.")

if __name__ == '__main__':
    main()