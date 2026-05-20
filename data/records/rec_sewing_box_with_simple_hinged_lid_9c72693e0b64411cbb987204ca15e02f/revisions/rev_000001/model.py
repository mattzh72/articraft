from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_sewing_box")

    dark_steel = Material("matte_dark_powder_coat", color=(0.08, 0.09, 0.09, 1.0))
    galvanized = Material("galvanized_reinforcement", color=(0.58, 0.61, 0.60, 1.0))
    hinge_steel = Material("brushed_hinge_steel", color=(0.72, 0.74, 0.72, 1.0))
    safety_yellow = Material("safety_yellow_guards", color=(1.0, 0.72, 0.05, 1.0))
    black_rubber = Material("black_rubber_gasket", color=(0.01, 0.01, 0.008, 1.0))
    warning_red = Material("red_lockout_pull", color=(0.80, 0.04, 0.02, 1.0))

    body = model.part("body")
    lid = model.part("lid")
    hasp = model.part("lockout_hasp")

    def cyl_rpy(axis: str) -> tuple[float, float, float]:
        if axis == "x":
            return (0.0, math.pi / 2.0, 0.0)
        if axis == "y":
            return (math.pi / 2.0, 0.0, 0.0)
        return (0.0, 0.0, 0.0)

    def add_cylinder(part, name, radius, length, xyz, axis="z", material=galvanized):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=cyl_rpy(axis)),
            material=material,
            name=name,
        )

    def add_bolt(part, name, xyz, face_axis="x", radius=0.007):
        add_cylinder(
            part,
            name,
            radius=radius,
            length=0.005,
            xyz=xyz,
            axis=face_axis,
            material=hinge_steel,
        )

    outer_l = 0.50
    outer_w = 0.32
    body_h = 0.20
    wall_t = 0.018
    floor_t = 0.018
    hinge_x = -0.258
    hinge_z = 0.235

    # Welded, open-top storage body.  The visible walls are deliberately
    # separate plate stock so the load paths and reinforced corners read as
    # industrial fabrication rather than a decorative case.
    body.visual(
        Box((outer_l, outer_w, floor_t)),
        origin=Origin(xyz=(0.0, 0.0, floor_t / 2.0)),
        material=dark_steel,
        name="floor",
    )
    body.visual(
        Box((wall_t, outer_w, body_h)),
        origin=Origin(xyz=(-outer_l / 2.0 + wall_t / 2.0, 0.0, body_h / 2.0)),
        material=dark_steel,
        name="rear_wall",
    )
    body.visual(
        Box((wall_t, outer_w, body_h)),
        origin=Origin(xyz=(outer_l / 2.0 - wall_t / 2.0, 0.0, body_h / 2.0)),
        material=dark_steel,
        name="front_wall",
    )
    body.visual(
        Box((outer_l, wall_t, body_h)),
        origin=Origin(xyz=(0.0, outer_w / 2.0 - wall_t / 2.0, body_h / 2.0)),
        material=dark_steel,
        name="side_wall_0",
    )
    body.visual(
        Box((outer_l, wall_t, body_h)),
        origin=Origin(xyz=(0.0, -outer_w / 2.0 + wall_t / 2.0, body_h / 2.0)),
        material=dark_steel,
        name="side_wall_1",
    )

    # Thin rubber top seal and welded lip plates, kept below the lid underside.
    body.visual(
        Box((0.035, outer_w + 0.018, 0.004)),
        origin=Origin(xyz=(outer_l / 2.0 - 0.016, 0.0, body_h + 0.0005)),
        material=black_rubber,
        name="front_gasket",
    )
    body.visual(
        Box((0.035, outer_w + 0.018, 0.004)),
        origin=Origin(xyz=(-outer_l / 2.0 + 0.016, 0.0, body_h + 0.0005)),
        material=black_rubber,
        name="rear_gasket",
    )
    body.visual(
        Box((outer_l, 0.020, 0.004)),
        origin=Origin(xyz=(0.0, outer_w / 2.0 - 0.007, body_h + 0.0005)),
        material=black_rubber,
        name="side_gasket_0",
    )
    body.visual(
        Box((outer_l, 0.020, 0.004)),
        origin=Origin(xyz=(0.0, -outer_w / 2.0 + 0.007, body_h + 0.0005)),
        material=black_rubber,
        name="side_gasket_1",
    )

    # Sewing-box interior: shallow welded dividers and spool pins are attached to
    # the floor so they do not read as loose contents.
    body.visual(
        Box((0.012, 0.255, 0.125)),
        origin=Origin(xyz=(-0.045, 0.0, floor_t + 0.062)),
        material=galvanized,
        name="divider_long",
    )
    body.visual(
        Box((0.155, 0.012, 0.100)),
        origin=Origin(xyz=(0.095, 0.045, floor_t + 0.050)),
        material=galvanized,
        name="divider_short",
    )
    for i, (x, y) in enumerate(((0.120, -0.075), (0.175, -0.075), (0.120, -0.020), (0.175, -0.020))):
        add_cylinder(body, f"spool_pin_{i}", 0.006, 0.082, (x, y, floor_t + 0.041), "z", galvanized)
        add_cylinder(body, f"spool_pin_base_{i}", 0.014, 0.006, (x, y, floor_t + 0.003), "z", galvanized)

    # Continuous rear hinge fixed leaf with alternating body-side knuckles.
    body.visual(
        Box((0.008, 0.286, 0.060)),
        origin=Origin(xyz=(hinge_x + 0.004, 0.0, hinge_z - 0.039)),
        material=hinge_steel,
        name="hinge_fixed_leaf",
    )
    for i, (y0, y1) in enumerate(((-0.136, -0.076), (-0.030, 0.030), (0.076, 0.136))):
        add_cylinder(
            body,
            f"hinge_body_knuckle_{i}",
            0.012,
            y1 - y0,
            (hinge_x, (y0 + y1) / 2.0, hinge_z),
            "y",
            hinge_steel,
        )
    add_cylinder(body, "hinge_pin_cap_0", 0.014, 0.012, (hinge_x, -0.142, hinge_z), "y", hinge_steel)
    add_cylinder(body, "hinge_pin_cap_1", 0.014, 0.012, (hinge_x, 0.142, hinge_z), "y", hinge_steel)
    for i, y in enumerate((-0.110, -0.045, 0.045, 0.110)):
        add_bolt(body, f"hinge_leaf_bolt_{i}", (hinge_x + 0.008, y, hinge_z - 0.043), "x", radius=0.006)

    # Safety guard rail over the rear pinch zone.  Feet tie into the rear wall;
    # upright posts tie into the rail so there are no suspended guard fragments.
    body.visual(
        Box((0.060, 0.055, 0.034)),
        origin=Origin(xyz=(-0.271, -0.153, 0.194)),
        material=safety_yellow,
        name="guard_foot_0",
    )
    body.visual(
        Box((0.060, 0.055, 0.034)),
        origin=Origin(xyz=(-0.271, 0.153, 0.194)),
        material=safety_yellow,
        name="guard_foot_1",
    )
    body.visual(
        Box((0.018, 0.020, 0.090)),
        origin=Origin(xyz=(-0.286, -0.153, 0.229)),
        material=safety_yellow,
        name="guard_post_0",
    )
    body.visual(
        Box((0.018, 0.020, 0.090)),
        origin=Origin(xyz=(-0.286, 0.153, 0.229)),
        material=safety_yellow,
        name="guard_post_1",
    )
    body.visual(
        Box((0.018, 0.348, 0.018)),
        origin=Origin(xyz=(-0.286, 0.0, 0.270)),
        material=safety_yellow,
        name="pinch_guard_rail",
    )

    # Over-travel blocks and reinforced rear load path near hinge roots.
    for i, y in enumerate((-0.112, 0.112)):
        body.visual(
            Box((0.020, 0.040, 0.046)),
            origin=Origin(xyz=(-0.302, y, 0.224)),
            material=safety_yellow,
            name=f"stop_block_{i}",
        )
        body.visual(
            Box((0.060, 0.018, 0.028)),
            origin=Origin(xyz=(-0.232, y, 0.180)),
            material=galvanized,
            name=f"rear_brace_{i}",
        )
        add_bolt(body, f"stop_bolt_{i}", (-0.314, y, 0.231), "x", radius=0.005)

    # Front lockout receiver with two cheeks.  The central slot remains clear for
    # the moving hasp plate; all cheeks are tied through the base plate.
    body.visual(
        Box((0.007, 0.118, 0.070)),
        origin=Origin(xyz=(0.2535, 0.0, 0.132)),
        material=hinge_steel,
        name="lockout_receiver_base",
    )
    body.visual(
        Box((0.014, 0.014, 0.064)),
        origin=Origin(xyz=(0.262, -0.052, 0.132)),
        material=safety_yellow,
        name="lockout_receiver_cheek_0",
    )
    body.visual(
        Box((0.014, 0.014, 0.064)),
        origin=Origin(xyz=(0.262, 0.052, 0.132)),
        material=safety_yellow,
        name="lockout_receiver_cheek_1",
    )
    for i, y in enumerate((-0.040, 0.040)):
        add_bolt(body, f"receiver_bolt_{i}", (0.258, y, 0.160), "x", radius=0.0055)

    # Corner armor and visible fasteners on the high-stress front corners.
    for i, y in enumerate((-0.162, 0.162)):
        body.visual(
            Box((0.045, 0.008, 0.110)),
            origin=Origin(xyz=(0.225, y, 0.105)),
            material=galvanized,
            name=f"corner_armor_{i}",
        )
        add_bolt(body, f"corner_bolt_low_{i}", (0.250, y, 0.060), "x", radius=0.005)
        add_bolt(body, f"corner_bolt_high_{i}", (0.250, y, 0.150), "x", radius=0.005)

    # Lid frame: heavy top plate with downturned safety lips and load ribs.
    lid.visual(
        Box((0.490, 0.350, 0.030)),
        origin=Origin(xyz=(0.265, 0.0, -0.014)),
        material=dark_steel,
        name="lid_panel",
    )
    lid.visual(
        Box((0.490, 0.010, 0.048)),
        origin=Origin(xyz=(0.265, 0.176, -0.039)),
        material=dark_steel,
        name="lid_side_lip_0",
    )
    lid.visual(
        Box((0.490, 0.010, 0.048)),
        origin=Origin(xyz=(0.265, -0.176, -0.039)),
        material=dark_steel,
        name="lid_side_lip_1",
    )
    lid.visual(
        Box((0.012, 0.350, 0.046)),
        origin=Origin(xyz=(0.515, 0.0, -0.035)),
        material=dark_steel,
        name="lid_front_lip",
    )
    lid.visual(
        Box((0.405, 0.026, 0.010)),
        origin=Origin(xyz=(0.285, 0.0, 0.006)),
        material=galvanized,
        name="lid_center_rib",
    )
    lid.visual(
        Box((0.040, 0.315, 0.010)),
        origin=Origin(xyz=(0.260, 0.0, 0.006)),
        material=galvanized,
        name="lid_cross_rib",
    )

    # Moving hinge leaf.  It is offset forward from the fixed knuckles and tied
    # to the alternate lid-side barrels through local straps.
    lid.visual(
        Box((0.070, 0.255, 0.006)),
        origin=Origin(xyz=(0.050, 0.0, 0.004)),
        material=hinge_steel,
        name="hinge_moving_leaf",
    )
    for i, (y0, y1) in enumerate(((-0.070, -0.034), (0.034, 0.070))):
        add_cylinder(lid, f"hinge_lid_knuckle_{i}", 0.0115, y1 - y0, (0.0, (y0 + y1) / 2.0, 0.0), "y", hinge_steel)
        lid.visual(
            Box((0.020, y1 - y0, 0.012)),
            origin=Origin(xyz=(0.013, (y0 + y1) / 2.0, 0.001)),
            material=hinge_steel,
            name=f"hinge_lid_bridge_{i}",
        )
        add_bolt(lid, f"moving_leaf_bolt_{i}", (0.064, (y0 + y1) / 2.0, 0.0095), "z", radius=0.005)

    # Lid-side stop horns approach the yellow stop blocks at the joint limit.
    for i, y in enumerate((-0.112, 0.112)):
        lid.visual(
            Box((0.030, 0.032, 0.018)),
            origin=Origin(xyz=(0.035, y, -0.025)),
            material=safety_yellow,
            name=f"lid_stop_horn_{i}",
        )

    # Hasp hinge ears are fixed to the front lip by standoff blocks.
    for i, y in enumerate((-0.038, 0.038)):
        lid.visual(
            Box((0.020, 0.020, 0.014)),
            origin=Origin(xyz=(0.525, y, -0.020)),
            material=hinge_steel,
            name=f"hasp_standoff_{i}",
        )
        add_cylinder(lid, f"hasp_lid_barrel_{i}", 0.006, 0.024, (0.532, y, -0.020), "y", hinge_steel)

    # Lockout hasp: a movable red plate with a raised black padlock-eye mark and
    # a central barrel captured between the fixed lid ears.
    add_cylinder(hasp, "hasp_barrel", 0.0055, 0.036, (0.0, 0.0, 0.0), "y", hinge_steel)
    hasp.visual(
        Box((0.007, 0.070, 0.112)),
        origin=Origin(xyz=(0.009, 0.0, -0.056)),
        material=warning_red,
        name="hasp_plate",
    )
    hasp.visual(
        Box((0.012, 0.092, 0.020)),
        origin=Origin(xyz=(0.010, 0.0, -0.105)),
        material=warning_red,
        name="lockout_tab",
    )
    add_cylinder(hasp, "padlock_eye_mark", 0.014, 0.003, (0.013, 0.0, -0.074), "x", black_rubber)
    add_bolt(hasp, "hasp_rivet_0", (0.014, -0.022, -0.028), "x", radius=0.004)
    add_bolt(hasp, "hasp_rivet_1", (0.014, 0.022, -0.028), "x", radius=0.004)

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=0.0, upper=1.55),
    )
    model.articulation(
        "lid_to_hasp",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=hasp,
        origin=Origin(xyz=(0.532, 0.0, -0.020)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hasp = object_model.get_part("lockout_hasp")
    lid_hinge = object_model.get_articulation("body_to_lid")
    hasp_hinge = object_model.get_articulation("lid_to_hasp")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="front_wall",
        min_gap=0.004,
        max_gap=0.010,
        name="closed lid clears top rim without crushing the box",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        elem_b="floor",
        min_overlap=0.30,
        name="lid covers the sewing-box footprint",
    )
    ctx.expect_gap(
        hasp,
        body,
        axis="x",
        positive_elem="hasp_plate",
        negative_elem="front_wall",
        min_gap=0.010,
        name="lockout hasp hangs outside front wall",
    )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lid_hinge: 1.55}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "hinged lid opens upward from rear barrel axis",
        closed_aabb is not None and open_aabb is not None and open_aabb[1][2] > closed_aabb[1][2] + 0.30,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    locked_aabb = ctx.part_element_world_aabb(hasp, elem="hasp_plate")
    with ctx.pose({hasp_hinge: 1.45}):
        raised_aabb = ctx.part_element_world_aabb(hasp, elem="hasp_plate")
    ctx.check(
        "lockout hasp rotates outward and upward",
        locked_aabb is not None and raised_aabb is not None and raised_aabb[0][2] > locked_aabb[0][2] + 0.045,
        details=f"locked={locked_aabb}, raised={raised_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
