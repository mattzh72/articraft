from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="a3_scanner")
    
    width = 0.55
    depth = 0.40
    height = 0.08
    t = 0.01
    
    body = model.part("body")
    
    # Body plates
    body.visual(Box((width, depth, t)), origin=Origin(xyz=(0, 0, -height/2 + t/2)), name="bottom_plate")
    body.visual(Box((t, depth, height - 2*t)), origin=Origin(xyz=(-width/2 + t/2, 0, 0)), name="left_plate")
    body.visual(Box((t, depth, height - 2*t)), origin=Origin(xyz=(width/2 - t/2, 0, 0)), name="right_plate")
    body.visual(Box((width - 2*t, t, height - 2*t)), origin=Origin(xyz=(0, -depth/2 + t/2, 0)), name="front_plate")
    body.visual(Box((width - 2*t, t, height - 2*t)), origin=Origin(xyz=(0, depth/2 - t/2, 0)), name="rear_plate")
    
    # Top plate with window
    window_w = 0.45
    window_d = 0.32
    
    w_tl = (width - window_w) / 2
    body.visual(Box((w_tl, depth, t)), origin=Origin(xyz=(-width/2 + w_tl/2, 0, height/2 - t/2)), name="top_left")
    body.visual(Box((w_tl, depth, t)), origin=Origin(xyz=(width/2 - w_tl/2, 0, height/2 - t/2)), name="top_right")
    
    d_tf = (depth - window_d) / 2
    body.visual(Box((window_w, d_tf, t)), origin=Origin(xyz=(0, -depth/2 + d_tf/2, height/2 - t/2)), name="top_front")
    body.visual(Box((window_w, d_tf, t)), origin=Origin(xyz=(0, depth/2 - d_tf/2, height/2 - t/2)), name="top_rear")
    
    # Glass pane
    body.visual(Box((window_w + 0.02, window_d + 0.02, 0.005)), origin=Origin(xyz=(0, 0, height/2 - t - 0.0025)), name="glass")
    
    # Hinge knuckles on body
    hinge_y = depth/2 + 0.015
    hinge_z = height/2
    knuckle_radius = 0.015
    knuckle_length = 0.03
    hinge_x1 = -0.15
    hinge_x2 = 0.15
    
    body.visual(Cylinder(radius=knuckle_radius, length=knuckle_length), origin=Origin(xyz=(hinge_x1, hinge_y, hinge_z), rpy=(0, 1.5708, 0)), name="body_knuckle_1")
    body.visual(Cylinder(radius=knuckle_radius, length=knuckle_length), origin=Origin(xyz=(hinge_x2, hinge_y, hinge_z), rpy=(0, 1.5708, 0)), name="body_knuckle_2")
    
    support_y_len = hinge_y - depth/2
    support_y_center = depth/2 + support_y_len/2
    body.visual(Box((knuckle_length, support_y_len, knuckle_radius*2)), origin=Origin(xyz=(hinge_x1, support_y_center, hinge_z)), name="body_knuckle_sup_1")
    body.visual(Box((knuckle_length, support_y_len, knuckle_radius*2)), origin=Origin(xyz=(hinge_x2, support_y_center, hinge_z)), name="body_knuckle_sup_2")
    
    # Guide rod for scan head
    rod_y = -0.10
    rod_z = -height/2 + t + 0.015
    body.visual(Cylinder(radius=0.005, length=width-2*t), origin=Origin(xyz=(0, rod_y, rod_z), rpy=(0, 1.5708, 0)), name="guide_rod")
    
    # Lid
    lid = model.part("lid")
    lid_thickness = 0.02
    lid.visual(Box((width, depth, lid_thickness)), origin=Origin(xyz=(0, -hinge_y, lid_thickness/2)), name="lid_panel")
    
    lid_knuckle_length = 0.02
    lid_x1_a = hinge_x1 - knuckle_length/2 - lid_knuckle_length/2
    lid_x1_b = hinge_x1 + knuckle_length/2 + lid_knuckle_length/2
    lid_x2_a = hinge_x2 - knuckle_length/2 - lid_knuckle_length/2
    lid_x2_b = hinge_x2 + knuckle_length/2 + lid_knuckle_length/2
    
    lid.visual(Cylinder(radius=knuckle_radius, length=lid_knuckle_length), origin=Origin(xyz=(lid_x1_a, 0, 0), rpy=(0, 1.5708, 0)), name="lid_knuckle_1a")
    lid.visual(Cylinder(radius=knuckle_radius, length=lid_knuckle_length), origin=Origin(xyz=(lid_x1_b, 0, 0), rpy=(0, 1.5708, 0)), name="lid_knuckle_1b")
    lid.visual(Cylinder(radius=knuckle_radius, length=lid_knuckle_length), origin=Origin(xyz=(lid_x2_a, 0, 0), rpy=(0, 1.5708, 0)), name="lid_knuckle_2a")
    lid.visual(Cylinder(radius=knuckle_radius, length=lid_knuckle_length), origin=Origin(xyz=(lid_x2_b, 0, 0), rpy=(0, 1.5708, 0)), name="lid_knuckle_2b")
    
    lid_sup_len = hinge_y - depth/2
    lid_sup_center_y = -lid_sup_len / 2
    
    lid.visual(Box((lid_knuckle_length, lid_sup_len, knuckle_radius*2)), origin=Origin(xyz=(lid_x1_a, lid_sup_center_y, 0)), name="lid_sup_1a")
    lid.visual(Box((lid_knuckle_length, lid_sup_len, knuckle_radius*2)), origin=Origin(xyz=(lid_x1_b, lid_sup_center_y, 0)), name="lid_sup_1b")
    lid.visual(Box((lid_knuckle_length, lid_sup_len, knuckle_radius*2)), origin=Origin(xyz=(lid_x2_a, lid_sup_center_y, 0)), name="lid_sup_2a")
    lid.visual(Box((lid_knuckle_length, lid_sup_len, knuckle_radius*2)), origin=Origin(xyz=(lid_x2_b, lid_sup_center_y, 0)), name="lid_sup_2b")
    
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=1.57),
    )
    
    # Scan head
    scan_head = model.part("scan_head")
    sh_x = 0.04
    sh_y = 0.36
    sh_z = 0.04
    
    scan_head.visual(Box((sh_x, sh_y, sh_z)), origin=Origin(xyz=(0, -rod_y, -rod_z)), name="scan_head_body")
    scan_head.visual(Box((sh_x, 0.02, 0.02)), origin=Origin(xyz=(0, 0, 0)), name="guide_sleeve")
    
    model.articulation(
        "scan_head_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=scan_head,
        origin=Origin(xyz=(0, rod_y, rod_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.5, lower=-0.24, upper=0.24),
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    scan_head = object_model.get_part("scan_head")
    
    ctx.allow_overlap(
        scan_head, body,
        elem_a="guide_sleeve", elem_b="guide_rod",
        reason="The guide sleeve wraps around the guide rod to slide along it."
    )
    
    ctx.allow_overlap(
        scan_head, body,
        elem_a="scan_head_body", elem_b="guide_rod",
        reason="The guide rod passes entirely through the scan head body."
    )
    
    # Tests
    ctx.expect_within(scan_head, body, axes="xyz", name="scan head remains inside body")
    
    ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.3, name="lid covers body")
    ctx.expect_gap(lid, body, axis="z", max_gap=0.001, max_penetration=0.001, positive_elem="lid_panel", negative_elem="top_left", name="lid rests on body")
    
    with ctx.pose({"scan_head_slide": 0.24}):
        ctx.expect_within(scan_head, body, axes="xyz", name="scan head remains inside body at max extension")
        
    with ctx.pose({"lid_hinge": 1.57}):
        ctx.expect_gap(lid, body, axis="z", min_gap=0.01, positive_elem="lid_panel", negative_elem="top_left", name="lid opens upward")
    
    return ctx.report()

object_model = build_object_model()
