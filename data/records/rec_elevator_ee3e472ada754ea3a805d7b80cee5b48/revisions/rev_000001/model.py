from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _helix_points(
    *,
    radius: float,
    height: float,
    pitch: float,
    turns_margin: float = 0.0,
    samples_per_turn: int = 18,
) -> list[tuple[float, float, float]]:
    turns = height / pitch
    steps = max(12, int(turns * samples_per_turn))
    points: list[tuple[float, float, float]] = []
    for i in range(steps + 1):
        t = i / steps
        angle = 2.0 * pi * turns * t
        points.append(
            (
                radius * cos(angle),
                radius * sin(angle),
                turns_margin + height * t,
            )
        )
    return points


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_drive_home_elevator")

    frame_mat = model.material("dark_powder_coat", rgba=(0.08, 0.09, 0.10, 1.0))
    car_mat = model.material("warm_white_car_panels", rgba=(0.78, 0.75, 0.68, 1.0))
    trim_mat = model.material("black_rubber_trim", rgba=(0.015, 0.015, 0.014, 1.0))
    glass_mat = model.material("pale_blue_glass", rgba=(0.45, 0.72, 0.90, 0.34))
    screw_mat = model.material("oiled_bright_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    hinge_mat = model.material("brushed_hinge_steel", rgba=(0.58, 0.58, 0.54, 1.0))
    safety_mat = model.material("yellow_safety_mark", rgba=(0.95, 0.72, 0.12, 1.0))

    # Fixed shaft frame: compact home-elevator guide tower with the screw column
    # mounted along the front-right side.
    frame = model.part("frame")
    frame.visual(Box((1.22, 0.86, 0.06)), origin=Origin(xyz=(0.0, 0.0, 0.03)), material=frame_mat, name="base_plate")
    frame.visual(Box((1.12, 0.80, 0.05)), origin=Origin(xyz=(0.0, 0.0, 3.03)), material=frame_mat, name="top_plate")
    for i, (x, y) in enumerate(
        (
            (-0.52, -0.38),
            (-0.52, 0.38),
            (0.52, -0.38),
            (0.52, 0.38),
        )
    ):
        frame.visual(
            Box((0.055, 0.055, 2.98)),
            origin=Origin(xyz=(x, y, 1.52)),
            material=frame_mat,
            name=f"corner_post_{i}",
        )
    frame.visual(Box((1.10, 0.04, 0.07)), origin=Origin(xyz=(0.0, -0.40, 0.12)), material=frame_mat, name="front_sill")
    frame.visual(Box((1.10, 0.04, 0.07)), origin=Origin(xyz=(0.0, -0.40, 2.62)), material=frame_mat, name="front_header")
    frame.visual(Box((0.06, 0.06, 2.58)), origin=Origin(xyz=(-0.47, 0.38, 1.48)), material=frame_mat, name="guide_rail_0")
    frame.visual(Box((0.06, 0.06, 2.58)), origin=Origin(xyz=(0.47, 0.38, 1.48)), material=frame_mat, name="guide_rail_1")

    screw_x = 0.57
    screw_y = -0.50
    screw_bottom = 0.20
    screw_height = 2.68
    for name, z in (("lower_bearing", screw_bottom), ("upper_bearing", screw_bottom + screw_height)):
        frame.visual(
            mesh_from_geometry(TorusGeometry(0.042, 0.010, radial_segments=24, tubular_segments=32), name),
            origin=Origin(xyz=(screw_x, screw_y, z)),
            material=hinge_mat,
            name=name,
        )
        frame.visual(
            Box((0.075, 0.17, 0.045)),
            origin=Origin(xyz=(0.49, -0.44, z)),
            material=frame_mat,
            name=f"{name}_bracket",
        )

    # Continuous lead screw with a real raised helical thread.
    screw = model.part("lead_screw")
    screw.visual(
        Cylinder(radius=0.022, length=screw_height + 0.04),
        origin=Origin(xyz=(0.0, 0.0, screw_height / 2.0)),
        material=screw_mat,
        name="screw_core",
    )
    screw.visual(
        Cylinder(radius=0.040, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, -0.0315)),
        material=screw_mat,
        name="lower_journal",
    )
    screw.visual(
        Cylinder(radius=0.040, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, screw_height + 0.0315)),
        material=screw_mat,
        name="upper_journal",
    )
    thread_mesh = tube_from_spline_points(
        _helix_points(radius=0.026, height=screw_height - 0.16, pitch=0.105, turns_margin=0.08, samples_per_turn=14),
        radius=0.006,
        samples_per_segment=2,
        radial_segments=8,
        cap_ends=True,
    )
    screw.visual(mesh_from_geometry(thread_mesh, "raised_thread"), material=screw_mat, name="raised_thread")
    model.articulation(
        "frame_to_lead_screw",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=screw,
        origin=Origin(xyz=(screw_x, screw_y, screw_bottom)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=12.0),
    )

    # The slim moving car is a connected, open-front cabin.  A split travelling
    # nut yoke on its front-right corner visually couples it to the lead screw.
    car = model.part("car")
    car_width = 0.76
    car_depth = 0.66
    car_height = 1.88
    wall = 0.035
    door_height = 1.62
    door_z = 0.15
    car.visual(Box((car_width, car_depth, 0.055)), origin=Origin(xyz=(0.0, 0.0, 0.0275)), material=car_mat, name="floor_pan")
    car.visual(Box((car_width, car_depth, 0.055)), origin=Origin(xyz=(0.0, 0.0, car_height - 0.0275)), material=car_mat, name="ceiling_pan")
    car.visual(Box((wall, car_depth, car_height)), origin=Origin(xyz=(-car_width / 2.0 + wall / 2.0, 0.0, car_height / 2.0)), material=car_mat, name="side_wall_0")
    car.visual(Box((wall, car_depth, car_height)), origin=Origin(xyz=(car_width / 2.0 - wall / 2.0, 0.0, car_height / 2.0)), material=car_mat, name="side_wall_1")
    car.visual(Box((car_width, wall, car_height)), origin=Origin(xyz=(0.0, car_depth / 2.0 - wall / 2.0, car_height / 2.0)), material=car_mat, name="rear_wall")
    car.visual(Box((0.055, 0.040, door_height + 0.20)), origin=Origin(xyz=(-0.36, -car_depth / 2.0 - 0.010, door_z + (door_height + 0.20) / 2.0)), material=trim_mat, name="front_jamb_0")
    car.visual(Box((0.055, 0.040, door_height + 0.20)), origin=Origin(xyz=(0.36, -car_depth / 2.0 - 0.010, door_z + (door_height + 0.20) / 2.0)), material=trim_mat, name="front_jamb_1")
    car.visual(Box((0.018, 0.085, door_height - 0.04)), origin=Origin(xyz=(-0.34, -car_depth / 2.0 - 0.0635, door_z + door_height / 2.0)), material=hinge_mat, name="fixed_jamb_leaf")
    car.visual(Box((0.78, 0.038, 0.070)), origin=Origin(xyz=(0.0, -car_depth / 2.0 - 0.010, door_z - 0.035)), material=trim_mat, name="door_threshold")
    car.visual(Box((0.78, 0.038, 0.070)), origin=Origin(xyz=(0.0, -car_depth / 2.0 - 0.010, door_z + door_height + 0.035)), material=trim_mat, name="door_header")
    car.visual(Box((0.40, 0.010, 0.56)), origin=Origin(xyz=(0.0, car_depth / 2.0 - wall - 0.0045, 1.12)), material=glass_mat, name="rear_window")
    car.visual(Box((0.010, 0.34, 0.48)), origin=Origin(xyz=(-car_width / 2.0 + wall + 0.0045, 0.02, 1.08)), material=glass_mat, name="side_window_0")
    car.visual(Box((0.010, 0.34, 0.48)), origin=Origin(xyz=(car_width / 2.0 - wall - 0.0045, 0.02, 1.08)), material=glass_mat, name="side_window_1")
    car.visual(Box((0.060, 0.090, 0.140)), origin=Origin(xyz=(-0.410, 0.345, 1.10)), material=trim_mat, name="guide_shoe_0")
    car.visual(Box((0.060, 0.090, 0.140)), origin=Origin(xyz=(0.410, 0.345, 1.10)), material=trim_mat, name="guide_shoe_1")
    car.visual(Box((0.030, 0.100, 0.105)), origin=Origin(xyz=(0.370, -0.375, 1.04)), material=frame_mat, name="front_nut_standoff")
    car.visual(Box((0.145, 0.090, 0.105)), origin=Origin(xyz=(0.455, -0.465, 1.04)), material=frame_mat, name="nut_support_arm")
    car.visual(Box((0.070, 0.145, 0.165)), origin=Origin(xyz=(0.488, screw_y, 1.04)), material=frame_mat, name="nut_yoke_spine")
    car.visual(Box((0.104, 0.034, 0.165)), origin=Origin(xyz=(screw_x, screw_y - 0.074, 1.04)), material=hinge_mat, name="nut_half_front")
    car.visual(Box((0.104, 0.034, 0.165)), origin=Origin(xyz=(screw_x, screw_y + 0.074, 1.04)), material=hinge_mat, name="nut_half_rear")
    car.visual(Box((0.13, 0.012, 0.035)), origin=Origin(xyz=(0.0, -car_depth / 2.0 - 0.030, door_z - 0.035)), material=safety_mat, name="yellow_warning_stripe")
    model.articulation(
        "frame_to_car",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=car,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=600.0, velocity=0.35, lower=0.0, upper=0.90),
    )

    # Front bifold door: panel 0 is hinged to the left car jamb; panel 1 is
    # hinged to panel 0 at the folding seam and mimics at twice the angle to form
    # a true accordion fold.
    panel_w = 0.34
    panel_body_w = panel_w - 0.030
    panel_t = 0.028
    hinge_r = 0.014
    hinge_y = -car_depth / 2.0 - 0.120
    panel_y = -car_depth / 2.0 - 0.105
    hinge_x = -panel_w

    door_0 = model.part("door_0")
    door_0.visual(
        Box((panel_body_w, panel_t, door_height)),
        origin=Origin(xyz=(panel_body_w / 2.0 + 0.012, panel_y - hinge_y, door_height / 2.0)),
        material=car_mat,
        name="panel_shell",
    )
    door_0.visual(Cylinder(radius=hinge_r, length=door_height + 0.04), origin=Origin(xyz=(0.0, 0.0, door_height / 2.0)), material=hinge_mat, name="jamb_barrel")
    door_0.visual(Box((0.018, 0.016, door_height - 0.06)), origin=Origin(xyz=(0.020, 0.008, door_height / 2.0)), material=hinge_mat, name="jamb_leaf")
    door_0.visual(Box((0.020, 0.016, door_height - 0.08)), origin=Origin(xyz=(panel_w - 0.010, 0.008, door_height / 2.0)), material=hinge_mat, name="seam_leaf")
    door_0.visual(Box((0.20, 0.010, 0.055)), origin=Origin(xyz=(panel_w * 0.55, 0.000, 0.95)), material=trim_mat, name="grab_rail")
    door_0.visual(Box((0.19, 0.006, 0.42)), origin=Origin(xyz=(panel_w * 0.50, 0.016, 1.10)), material=glass_mat, name="narrow_window")
    model.articulation(
        "car_to_door_0",
        ArticulationType.REVOLUTE,
        parent=car,
        child=door_0,
        origin=Origin(xyz=(hinge_x, hinge_y, door_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.15),
    )

    door_1 = model.part("door_1")
    door_1.visual(
        Box((panel_body_w, panel_t, door_height)),
        origin=Origin(xyz=(panel_body_w / 2.0 + 0.012, panel_y - hinge_y, door_height / 2.0)),
        material=car_mat,
        name="panel_shell",
    )
    door_1.visual(Cylinder(radius=hinge_r, length=door_height + 0.04), origin=Origin(xyz=(0.0, 0.0, door_height / 2.0)), material=hinge_mat, name="fold_barrel")
    door_1.visual(Box((0.018, 0.016, door_height - 0.06)), origin=Origin(xyz=(0.020, 0.008, door_height / 2.0)), material=hinge_mat, name="fold_leaf")
    door_1.visual(Box((0.20, 0.010, 0.055)), origin=Origin(xyz=(panel_w * 0.45, 0.000, 0.95)), material=trim_mat, name="grab_rail")
    door_1.visual(Box((0.19, 0.006, 0.42)), origin=Origin(xyz=(panel_w * 0.50, 0.016, 1.10)), material=glass_mat, name="narrow_window")
    model.articulation(
        "door_0_to_door_1",
        ArticulationType.REVOLUTE,
        parent=door_0,
        child=door_1,
        origin=Origin(xyz=(panel_w, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=0.0, upper=2.30),
        mimic=Mimic(joint="car_to_door_0", multiplier=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    car = object_model.get_part("car")
    frame = object_model.get_part("frame")
    screw = object_model.get_part("lead_screw")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    lift = object_model.get_articulation("frame_to_car")
    fold = object_model.get_articulation("car_to_door_0")

    ctx.allow_overlap(
        door_0,
        door_1,
        elem_a="seam_leaf",
        elem_b="fold_barrel",
        reason="The bifold center hinge barrel is intentionally seated through the fixed seam leaf.",
    )
    ctx.expect_gap(
        door_1,
        door_0,
        axis="x",
        positive_elem="fold_barrel",
        negative_elem="seam_leaf",
        max_penetration=0.016,
        name="fold hinge barrel is captured by the seam leaf",
    )

    ctx.expect_within(
        car,
        frame,
        axes="xy",
        margin=0.04,
        name="car rides inside the guide frame footprint",
    )
    ctx.expect_overlap(
        screw,
        frame,
        axes="z",
        min_overlap=2.55,
        elem_a="screw_core",
        name="lead screw spans between the bearing plates",
    )

    rest_pos = ctx.part_world_position(car)
    with ctx.pose({lift: 0.90}):
        raised_pos = ctx.part_world_position(car)
        ctx.expect_within(
            car,
            frame,
            axes="xy",
            margin=0.04,
            name="raised car remains captured by the guide frame",
        )
    ctx.check(
        "car lift joint moves vertically",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.80,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    with ctx.pose({fold: 0.0}):
        ctx.expect_gap(
            door_1,
            door_0,
            axis="x",
            positive_elem="panel_shell",
            negative_elem="panel_shell",
            min_gap=0.0,
            max_gap=0.04,
            name="closed bifold panels meet at the center seam",
        )
        closed_seam = ctx.part_world_position(door_1)

    with ctx.pose({fold: 1.0}):
        open_seam = ctx.part_world_position(door_1)
        ctx.expect_overlap(
            door_0,
            door_1,
            axes="z",
            min_overlap=1.50,
            elem_a="panel_shell",
            elem_b="panel_shell",
            name="both bifold panels remain full height while folding",
        )
    ctx.check(
        "bifold seam folds outward at the car front",
        closed_seam is not None and open_seam is not None and open_seam[1] < closed_seam[1] - 0.18,
        details=f"closed_seam={closed_seam}, open_seam={open_seam}",
    )

    return ctx.report()


object_model = build_object_model()
