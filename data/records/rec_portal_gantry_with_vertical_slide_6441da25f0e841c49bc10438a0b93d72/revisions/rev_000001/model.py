from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="portal_gantry_vertical_slide_study")

    painted = model.material("painted_cast_steel", color=(0.30, 0.33, 0.34, 1.0))
    dark_plate = model.material("dark_blued_plate", color=(0.08, 0.09, 0.095, 1.0))
    rail_steel = model.material("ground_linear_rail", color=(0.78, 0.80, 0.78, 1.0))
    black = model.material("black_oxide_fasteners", color=(0.015, 0.015, 0.014, 1.0))
    bronze = model.material("oilite_bearing_bronze", color=(0.78, 0.50, 0.22, 1.0))
    warning = model.material("satin_access_cover", color=(0.18, 0.20, 0.21, 1.0))

    def add_box(part, name, size, xyz, material, rpy=(0.0, 0.0, 0.0)):
        part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    def add_cylinder(part, name, radius, length, xyz, material, rpy=(0.0, 0.0, 0.0)):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=material,
            name=name,
        )

    frame = model.part("frame")

    # Portal feet and columns: broad fabricated base plates, boxed uprights, and
    # visible anchor hardware.  The whole frame is authored as one rigid welded
    # reference link so the Z carriage can slide beneath a fixed bridge.
    for sx, label in ((-1.0, "left"), (1.0, "right")):
        x = sx * 0.72
        add_box(frame, f"{label}_foot_plate", (0.38, 0.62, 0.08), (x, 0.0, 0.04), painted)
        add_box(frame, f"{label}_column", (0.14, 0.26, 1.48), (x, 0.0, 0.82), painted)
        add_box(frame, f"{label}_front_wear_strip", (0.028, 0.018, 1.28), (x, -0.149, 0.82), dark_plate)
        add_box(frame, f"{label}_rear_wear_strip", (0.028, 0.018, 1.28), (x, 0.149, 0.82), dark_plate)
        add_box(frame, f"{label}_top_knee", (0.26, 0.30, 0.10), (x, 0.0, 1.50), painted)
        add_box(frame, f"{label}_front_gusset", (0.48, 0.045, 0.055), (x - sx * 0.10, -0.155, 1.36), painted, rpy=(0.0, sx * 0.70, 0.0))
        add_box(frame, f"{label}_rear_gusset", (0.48, 0.045, 0.055), (x - sx * 0.10, 0.155, 1.36), painted, rpy=(0.0, sx * 0.70, 0.0))

        for bx in (-0.12, 0.12):
            for by in (-0.22, 0.22):
                add_cylinder(
                    frame,
                    f"{label}_anchor_{bx:+.2f}_{by:+.2f}",
                    0.022,
                    0.014,
                    (x + bx, by, 0.087),
                    black,
                )

    # Crossbeam fabricated as a box beam with bolted front/back cover plates and
    # cap plates, deliberately unstyled and machine-study-like.
    add_box(frame, "crossbeam_web", (1.78, 0.24, 0.22), (0.0, 0.0, 1.65), painted)
    frame.visual(
        Box((1.84, 0.035, 0.34)),
        origin=Origin(xyz=(0.0, -0.130, 1.65)),
        material=dark_plate,
        name="front_beam_plate",
    )
    add_box(frame, "rear_beam_plate", (1.84, 0.035, 0.34), (0.0, 0.130, 1.65), dark_plate)
    add_box(frame, "top_cap_plate", (1.88, 0.34, 0.05), (0.0, 0.0, 1.805), painted)
    add_box(frame, "bottom_cap_plate", (1.82, 0.30, 0.05), (0.0, 0.0, 1.495), painted)
    add_box(frame, "front_lower_tie", (1.42, 0.060, 0.070), (0.0, -0.255, 0.34), painted)
    add_box(frame, "rear_lower_tie", (1.42, 0.060, 0.070), (0.0, 0.255, 0.34), painted)
    for x in (-0.72, 0.72):
        add_box(frame, f"front_tie_stand_{x:+.2f}", (0.10, 0.065, 0.30), (x, -0.255, 0.19), painted)
        add_box(frame, f"rear_tie_stand_{x:+.2f}", (0.10, 0.065, 0.30), (x, 0.255, 0.19), painted)

    for x in (-0.42, -0.21, 0.21, 0.42):
        add_cylinder(frame, f"front_beam_bolt_{x:+.2f}", 0.014, 0.010, (x, -0.145, 1.72), black, rpy=(pi / 2, 0.0, 0.0))
        add_cylinder(frame, f"rear_beam_bolt_{x:+.2f}", 0.014, 0.010, (x, 0.145, 1.72), black, rpy=(pi / 2, 0.0, 0.0))

    # Central guide module suspended below the bridge.  Two ground rods, a lead
    # screw line, guard strips, and split bearing saddles are all exposed.
    add_box(frame, "top_rail_clamp", (0.74, 0.19, 0.07), (0.0, -0.160, 1.405), dark_plate)
    frame.visual(
        Box((0.78, 0.19, 0.07)),
        origin=Origin(xyz=(0.0, -0.160, 0.300)),
        material=dark_plate,
        name="lower_rail_support",
    )
    for x in (-0.28, 0.28):
        add_box(frame, f"clamp_hanger_{x:+.2f}", (0.09, 0.12, 0.08), (x, -0.160, 1.455), painted)

    for x, label in ((-0.25, "left"), (0.25, "right")):
        if label == "left":
            frame.visual(
                Cylinder(radius=0.016, length=1.035),
                origin=Origin(xyz=(x, -0.160, 0.8525)),
                material=rail_steel,
                name="left_guide_rod",
            )
        else:
            frame.visual(
                Cylinder(radius=0.016, length=1.035),
                origin=Origin(xyz=(x, -0.160, 0.8525)),
                material=rail_steel,
                name="right_guide_rod",
            )
        add_box(frame, f"{label}_rail_guard", (0.024, 0.018, 1.035), (x + (-0.075 if x < 0 else 0.075), -0.205, 0.8525), black)
        add_box(frame, f"{label}_top_split_clamp", (0.095, 0.055, 0.050), (x, -0.205, 1.355), dark_plate)
        add_box(frame, f"{label}_bottom_split_clamp", (0.095, 0.055, 0.050), (x, -0.205, 0.350), dark_plate)

    add_box(frame, "top_screw_saddle", (0.115, 0.055, 0.055), (0.0, -0.205, 1.355), dark_plate)
    add_box(frame, "bottom_screw_saddle", (0.115, 0.055, 0.055), (0.0, -0.205, 0.350), dark_plate)
    for x in (-0.035, 0.035):
        add_box(frame, f"top_screw_split_{x:+.2f}", (0.026, 0.047, 0.080), (x, -0.160, 1.355), dark_plate)
        add_box(frame, f"bottom_screw_split_{x:+.2f}", (0.026, 0.047, 0.080), (x, -0.160, 0.350), dark_plate)
    add_box(frame, "top_journal_pad", (0.055, 0.010, 0.045), (0.0, -0.181, 1.320), bronze)
    add_box(frame, "bottom_journal_pad", (0.055, 0.010, 0.045), (0.0, -0.181, 0.372), bronze)

    lead_screw = model.part("lead_screw")
    add_cylinder(lead_screw, "threaded_shaft", 0.011, 1.025, (0.0, 0.0, 0.0), rail_steel)
    for z in (-0.46, 0.46):
        add_cylinder(lead_screw, f"end_journal_{z:+.2f}", 0.016, 0.055, (0.0, 0.0, z), rail_steel)
    model.articulation(
        "frame_to_lead_screw",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=lead_screw,
        origin=Origin(xyz=(0.0, -0.160, 0.8525)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=18.0),
    )

    carriage = model.part("carriage")
    # A compact Z saddle hangs from the bridge.  The plate, bearing packs,
    # nut-carrier halves, and tool pad are intentionally machined and exposed.
    carriage.visual(
        Box((0.46, 0.060, 0.48)),
        origin=Origin(xyz=(0.0, -0.258, -0.070)),
        material=dark_plate,
        name="main_plate",
    )
    add_box(carriage, "top_yoke", (0.42, 0.105, 0.055), (0.0, -0.228, 0.135), painted)
    add_box(carriage, "lower_tool_pad", (0.24, 0.060, 0.18), (0.0, -0.340, -0.360), dark_plate)
    add_box(carriage, "tool_pad_web", (0.16, 0.060, 0.15), (0.0, -0.295, -0.265), painted)
    add_box(carriage, "central_rib", (0.055, 0.080, 0.42), (0.0, -0.214, -0.050), painted)

    for x, label in ((-0.25, "left"), (0.25, "right")):
        for z, row in ((0.080, "upper"), (-0.185, "lower")):
            if label == "left" and row == "upper":
                carriage.visual(
                    Box((0.090, 0.035, 0.115)),
                    origin=Origin(xyz=(x, -0.205, z)),
                    material=painted,
                    name="left_upper_bearing_back",
                )
                carriage.visual(
                    Box((0.070, 0.008, 0.100)),
                    origin=Origin(xyz=(x, -0.180, z)),
                    material=bronze,
                    name="left_upper_wear_pad",
                )
            elif label == "left" and row == "lower":
                carriage.visual(
                    Box((0.090, 0.035, 0.115)),
                    origin=Origin(xyz=(x, -0.205, z)),
                    material=painted,
                    name="left_lower_bearing_back",
                )
                add_box(carriage, f"{label}_{row}_wear_pad", (0.070, 0.008, 0.100), (x, -0.180, z), bronze)
            elif label == "right" and row == "upper":
                add_box(carriage, f"{label}_{row}_bearing_back", (0.090, 0.035, 0.115), (x, -0.205, z), painted)
                carriage.visual(
                    Box((0.070, 0.008, 0.100)),
                    origin=Origin(xyz=(x, -0.180, z)),
                    material=bronze,
                    name="right_upper_wear_pad",
                )
            else:
                add_box(carriage, f"{label}_{row}_bearing_back", (0.090, 0.035, 0.115), (x, -0.205, z), painted)
                add_box(carriage, f"{label}_{row}_wear_pad", (0.070, 0.008, 0.100), (x, -0.180, z), bronze)
            add_box(carriage, f"{label}_{row}_inner_cheek", (0.018, 0.047, 0.115), (x + (-0.034 if x < 0 else 0.034), -0.166, z), painted)
            add_box(carriage, f"{label}_{row}_outer_cheek", (0.018, 0.047, 0.115), (x + (0.034 if x < 0 else -0.034), -0.166, z), painted)
            add_box(carriage, f"{label}_{row}_bearing_web", (0.070, 0.014, 0.095), (x, -0.224, z), painted)
            add_cylinder(carriage, f"{label}_{row}_clamp_screw", 0.010, 0.010, (x, -0.225, z + 0.030), black, rpy=(pi / 2, 0.0, 0.0))

    for x, label in ((-0.032, "left"), (0.032, "right")):
        add_box(carriage, f"{label}_nut_half", (0.024, 0.050, 0.120), (x, -0.160, -0.055), bronze)
        add_box(carriage, f"{label}_nut_web", (0.020, 0.065, 0.100), (x, -0.210, -0.055), painted)
    add_box(carriage, "nut_cross_clamp", (0.110, 0.020, 0.030), (0.0, -0.235, 0.020), painted)

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 1.120)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=700.0, velocity=0.35, lower=0.0, upper=0.42),
    )

    bridge_cover = model.part("bridge_cover")
    bridge_cover.visual(
        Box((0.52, 0.012, 0.15)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=warning,
        name="cover_plate",
    )
    add_box(bridge_cover, "pull_land", (0.20, 0.008, 0.035), (0.0, -0.010, 0.0), black)
    for x in (-0.22, 0.22):
        for z in (-0.055, 0.055):
            add_cylinder(bridge_cover, f"socket_screw_{x:+.2f}_{z:+.2f}", 0.010, 0.008, (x, -0.010, z), black, rpy=(pi / 2, 0.0, 0.0))
    model.articulation(
        "frame_to_bridge_cover",
        ArticulationType.FIXED,
        parent=frame,
        child=bridge_cover,
        origin=Origin(xyz=(0.0, -0.1535, 1.575)),
    )

    carriage_cover = model.part("carriage_cover")
    carriage_cover.visual(
        Box((0.25, 0.010, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=warning,
        name="cover_plate",
    )
    add_box(carriage_cover, "inspection_slot", (0.16, 0.006, 0.028), (0.0, -0.008, 0.0), black)
    for x in (-0.095, 0.095):
        for z in (-0.085, 0.085):
            add_cylinder(carriage_cover, f"socket_screw_{x:+.2f}_{z:+.2f}", 0.008, 0.007, (x, -0.003, z), black, rpy=(pi / 2, 0.0, 0.0))
    model.articulation(
        "carriage_to_cover",
        ArticulationType.FIXED,
        parent=carriage,
        child=carriage_cover,
        origin=Origin(xyz=(0.0, -0.293, -0.070)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    bridge_cover = object_model.get_part("bridge_cover")
    carriage_cover = object_model.get_part("carriage_cover")
    slide = object_model.get_articulation("frame_to_carriage")
    screw_joint = object_model.get_articulation("frame_to_lead_screw")

    ctx.check(
        "carriage uses vertical prismatic travel",
        slide.articulation_type == ArticulationType.PRISMATIC and tuple(slide.axis) == (0.0, 0.0, -1.0),
        details=f"type={slide.articulation_type}, axis={slide.axis}",
    )
    ctx.check(
        "lead screw is a rotating joint",
        screw_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(screw_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={screw_joint.articulation_type}, axis={screw_joint.axis}",
    )

    ctx.expect_gap(
        frame,
        carriage,
        axis="y",
        positive_elem="left_guide_rod",
        negative_elem="left_upper_wear_pad",
        max_gap=0.0005,
        max_penetration=0.0005,
        name="left upper bearing is preloaded on guide",
    )
    ctx.expect_gap(
        frame,
        carriage,
        axis="y",
        positive_elem="right_guide_rod",
        negative_elem="right_upper_wear_pad",
        max_gap=0.0005,
        max_penetration=0.0005,
        name="right upper bearing is preloaded on guide",
    )
    ctx.expect_overlap(
        carriage,
        frame,
        axes="z",
        elem_a="left_upper_bearing_back",
        elem_b="left_guide_rod",
        min_overlap=0.10,
        name="upper bearing lies on guide span",
    )
    ctx.expect_gap(
        frame,
        bridge_cover,
        axis="y",
        positive_elem="front_beam_plate",
        negative_elem="cover_plate",
        max_gap=0.001,
        max_penetration=0.000001,
        name="bridge access cover seats on beam plate",
    )
    ctx.expect_gap(
        carriage,
        carriage_cover,
        axis="y",
        positive_elem="main_plate",
        negative_elem="cover_plate",
        max_gap=0.001,
        max_penetration=0.000001,
        name="carriage access cover seats on saddle plate",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.42}):
        extended_pos = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            frame,
            axes="z",
            elem_a="left_lower_bearing_back",
            elem_b="left_guide_rod",
            min_overlap=0.10,
            name="lower bearing remains on guide at full stroke",
        )
        ctx.expect_gap(
            carriage,
            frame,
            axis="z",
            positive_elem="main_plate",
            negative_elem="lower_rail_support",
            min_gap=0.015,
            name="carriage clears lower rail support at full stroke",
        )

    ctx.check(
        "positive slide command lowers carriage",
        rest_pos is not None and extended_pos is not None and extended_pos[2] < rest_pos[2] - 0.35,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
