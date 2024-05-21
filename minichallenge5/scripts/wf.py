import numpy as np

def compute_wf_controller(closest_angle, clk_cnt):
        theta_ao = closest_angle
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

        theta_ao = theta_ao - np.pi
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

        theta_fw = -np.pi/2 + theta_ao if clk_cnt else np.pi/2 + theta_ao
        theta_fw = np.arctan2(np.sin(theta_fw), np.cos(theta_fw))

        w_fw = 3.0 * theta_fw
        v_fw = 0.07
    
        return v_fw, w_fw