class ConfigPlanner():
    use_quintic = True
    max_heading_rate = 10
    a_star_cost_threshold = 20.0
    step_cost_weight = 0.50
    optimizer_cost_threshold = 10.0
    safe_cost_margin = optimizer_cost_threshold
    use_clearance_cost = True
    clearance_cost_mode = "relative"
    clearance_cost_weight = 20.0
    clearance_cost_local_radius = 2.0
    clearance_cost_cap = 20.0
    clearance_cost_decay = 1.0


class ConfigWrapper():
    tomo_dir = '/rsc/tomogram/'


class Config():
    planner = ConfigPlanner()
    wrapper = ConfigWrapper()
