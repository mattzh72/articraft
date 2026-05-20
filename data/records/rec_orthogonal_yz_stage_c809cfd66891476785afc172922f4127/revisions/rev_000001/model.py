from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_mounted_transfer_stage")

    painted = model.material("graphite_powder_coat", rgba=(0.12, 0.13, 0.14, 1.0))
    dark = model.material("dark_cast_iron", rgba=(0.05, 0.055, 0.06, 1.0))
    rail = model.material("ground_steel_rails", rgba=(0.70, 0.72, 0.70, 1.0))
    carriage_blue = model.material("blue_anodized_carriage", rgba=(0.05, 0.18, 0.34, 1.0))
    tool_yellow = model.material("safety_yellow_tooling", rgba=(0.92, 0.62, 0.10, 1.0))
    wear = model.material("black_polymer_wear", rgba=(0.01, 0.012, 0.014, 1.0))
    fastener = model.material("dark_socket_screws", rgba=(0.02, 0.02, 0.022, 1.0))

    frame = model.part("frame")

    # Grounded back tower and base.  The long backing plate carries the lateral
    # Y rails, while the tower and base make the side-mounted stage read as a
    # rigid piece of automation equipment rather than a floating rail set.
    frame.visual(
        Box((0.34, 1.65, 0.06)),
        origin=Origin(xyz=(-0.05, 0.0, 0.03)),
        material=dark,
        name="base_foot",
    )
    frame.visual(
        Box((0.16, 0.30, 1.32)),
        origin=Origin(xyz=(-0.08, 0.0, 0.72)),
        material=painted,
        name="back_tower",
    )
    frame.visual(
        Box((0.06, 1.48, 0.48)),
        origin=Origin(xyz=(-0.03, 0.0, 0.93)),
        material=painted,
        name="rail_backing_plate",
    )
    frame.visual(
        Box((0.05, 0.12, 1.22)),
        origin=Origin(xyz=(-0.155, -0.18, 0.67)),
        material=dark,
        name="tower_side_rib_0",
    )
    frame.visual(
        Box((0.05, 0.12, 1.22)),
        origin=Origin(xyz=(-0.155, 0.18, 0.67)),
        material=dark,
        name="tower_side_rib_1",
    )
    frame.visual(
        Box((0.15, 0.08, 0.58)),
        origin=Origin(xyz=(-0.075, -0.76, 0.88)),
        material=painted,
        name="end_standard_0",
    )
    frame.visual(
        Box((0.15, 0.08, 0.58)),
        origin=Origin(xyz=(-0.075, 0.76, 0.88)),
        material=painted,
        name="end_standard_1",
    )

    # Fixed Y-axis linear rails and hard stops.
    frame.visual(
        Box((0.04, 1.34, 0.035)),
        origin=Origin(xyz=(0.02, 0.0, 1.08)),
        material=rail,
        name="upper_y_rail",
    )
    frame.visual(
        Box((0.04, 1.34, 0.035)),
        origin=Origin(xyz=(0.02, 0.0, 0.78)),
        material=rail,
        name="lower_y_rail",
    )
    for y, suffix in [(-0.62, "near"), (0.62, "far")]:
        frame.visual(
            Box((0.05, 0.055, 0.095)),
            origin=Origin(xyz=(0.06, y, 1.08)),
            material=dark,
            name=f"upper_y_stop_{suffix}",
        )
        frame.visual(
            Box((0.05, 0.055, 0.095)),
            origin=Origin(xyz=(0.06, y, 0.78)),
            material=dark,
            name=f"lower_y_stop_{suffix}",
        )

    # Cast pads and a cable tray tie the rail plate visually back into the tower.
    frame.visual(
        Box((0.08, 1.24, 0.045)),
        origin=Origin(xyz=(-0.04, 0.0, 1.235)),
        material=dark,
        name="top_tie_beam",
    )
    frame.visual(
        Box((0.08, 1.24, 0.045)),
        origin=Origin(xyz=(-0.04, 0.0, 0.625)),
        material=dark,
        name="lower_tie_beam",
    )

    crosshead = model.part("crosshead")

    # The carriage sits in front of the fixed rails with small running
    # clearance.  Guide bodies, flanges, and polymer strips overlap in Z/Y with
    # the rails but stay in front of them along X, making the Y path clear while
    # still reading as a captured carriage.
    crosshead.visual(
        Box((0.04, 0.28, 0.44)),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material=carriage_blue,
        name="carriage_web",
    )
    crosshead.visual(
        Box((0.055, 0.235, 0.075)),
        origin=Origin(xyz=(0.0715, 0.0, 0.15)),
        material=carriage_blue,
        name="upper_bearing_body",
    )
    crosshead.visual(
        Box((0.055, 0.235, 0.075)),
        origin=Origin(xyz=(0.0715, 0.0, -0.15)),
        material=carriage_blue,
        name="lower_bearing_body",
    )
    for zc, rail_name in [(0.15, "upper"), (-0.15, "lower")]:
        crosshead.visual(
            Box((0.021, 0.25, 0.015)),
            origin=Origin(xyz=(0.0445, 0.0, zc + 0.050)),
            material=wear,
            name=f"{rail_name}_top_wear_strip",
        )
        crosshead.visual(
            Box((0.021, 0.25, 0.015)),
            origin=Origin(xyz=(0.0445, 0.0, zc - 0.050)),
            material=wear,
            name=f"{rail_name}_bottom_wear_strip",
        )
        crosshead.visual(
            Box((0.018, 0.25, 0.038)),
            origin=Origin(xyz=(0.049, 0.0, zc)),
            material=wear,
            name=f"{rail_name}_front_wiper",
        )

    crosshead.visual(
        Box((0.14, 0.30, 0.06)),
        origin=Origin(xyz=(0.13, 0.0, 0.14)),
        material=carriage_blue,
        name="z_axis_top_flange",
    )
    crosshead.visual(
        Box((0.11, 0.28, 0.05)),
        origin=Origin(xyz=(0.13, 0.0, -0.40)),
        material=carriage_blue,
        name="z_axis_lower_flange",
    )
    crosshead.visual(
        Box((0.025, 0.025, 0.90)),
        origin=Origin(xyz=(0.165, -0.075, -0.22)),
        material=rail,
        name="z_guide_0",
    )
    crosshead.visual(
        Box((0.025, 0.025, 0.90)),
        origin=Origin(xyz=(0.165, 0.075, -0.22)),
        material=rail,
        name="z_guide_1",
    )
    crosshead.visual(
        Box((0.05, 0.06, 0.045)),
        origin=Origin(xyz=(0.135, -0.075, 0.18)),
        material=dark,
        name="upper_z_stop_0",
    )
    crosshead.visual(
        Box((0.05, 0.06, 0.045)),
        origin=Origin(xyz=(0.135, 0.075, 0.18)),
        material=dark,
        name="upper_z_stop_1",
    )
    crosshead.visual(
        Box((0.05, 0.06, 0.045)),
        origin=Origin(xyz=(0.135, -0.075, -0.62)),
        material=dark,
        name="lower_z_stop_0",
    )
    crosshead.visual(
        Box((0.05, 0.06, 0.045)),
        origin=Origin(xyz=(0.135, 0.075, -0.62)),
        material=dark,
        name="lower_z_stop_1",
    )

    tool_plate = model.part("tool_plate")

    # The descending plate carries four guide shoes plus small keeper/wear
    # faces.  They remain on the two Z rails throughout travel, so the plate
    # reads as captured by the crosshead rather than simply hanging below it.
    tool_plate.visual(
        Box((0.045, 0.26, 0.30)),
        origin=Origin(xyz=(0.055, 0.0, -0.27)),
        material=tool_yellow,
        name="tool_plate_face",
    )
    tool_plate.visual(
        Box((0.07, 0.22, 0.055)),
        origin=Origin(xyz=(0.055, 0.0, -0.43)),
        material=tool_yellow,
        name="tool_mount_bar",
    )
    tool_plate.visual(
        Box((0.055, 0.14, 0.055)),
        origin=Origin(xyz=(0.075, 0.0, -0.485)),
        material=dark,
        name="tool_adapter",
    )
    def add_guide_shoe(y: float, z: float, shoe_name: str, wear_name: str) -> None:
        tool_plate.visual(
            Box((0.035, 0.046, 0.085)),
            origin=Origin(xyz=(0.035, y, z)),
            material=tool_yellow,
            name=shoe_name,
        )
        tool_plate.visual(
            Box((0.012, 0.056, 0.070)),
            origin=Origin(xyz=(0.019, y, z)),
            material=wear,
            name=wear_name,
        )

    add_guide_shoe(-0.075, -0.18, "upper_guide_shoe_0", "upper_wear_face_0")
    add_guide_shoe(-0.075, -0.34, "lower_guide_shoe_0", "lower_wear_face_0")
    add_guide_shoe(0.075, -0.18, "upper_guide_shoe_1", "upper_wear_face_1")
    add_guide_shoe(0.075, -0.34, "lower_guide_shoe_1", "lower_wear_face_1")
    for y, suffix in [(-0.095, "0"), (0.095, "1")]:
        tool_plate.visual(
            Box((0.012, 0.03, 0.25)),
            origin=Origin(xyz=(0.083, y, -0.27)),
            material=fastener,
            name=f"side_wear_strip_{suffix}",
        )

    y_slide = model.articulation(
        "frame_to_crosshead",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=crosshead,
        origin=Origin(xyz=(0.0, -0.30, 0.93)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.55, lower=0.0, upper=0.60),
    )
    z_slide = model.articulation(
        "crosshead_to_tool_plate",
        ArticulationType.PRISMATIC,
        parent=crosshead,
        child=tool_plate,
        origin=Origin(xyz=(0.165, 0.0, 0.05)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.35, lower=0.0, upper=0.30),
    )

    # Pose samples exercise both usable ends of both orthogonal stages.
    y_slide.meta["qc_samples"] = [0.0, 0.30, 0.60]
    z_slide.meta["qc_samples"] = [0.0, 0.15, 0.30]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    crosshead = object_model.get_part("crosshead")
    tool_plate = object_model.get_part("tool_plate")
    y_slide = object_model.get_articulation("frame_to_crosshead")
    z_slide = object_model.get_articulation("crosshead_to_tool_plate")

    # Running clearance to the fixed Y rails at the carriage bearing bodies.
    for bearing, rail_elem in [
        ("upper_bearing_body", "upper_y_rail"),
        ("lower_bearing_body", "lower_y_rail"),
    ]:
        ctx.expect_gap(
            crosshead,
            frame,
            axis="x",
            min_gap=0.003,
            positive_elem=bearing,
            negative_elem=rail_elem,
            name=f"{bearing} clears its fixed rail",
        )
        ctx.expect_overlap(
            crosshead,
            frame,
            axes="yz",
            min_overlap=0.03,
            elem_a=bearing,
            elem_b=rail_elem,
            name=f"{bearing} remains aligned on its rail",
        )

    # The tool guide shoes stay in front of the Z rails and remain engaged at
    # both ends of vertical travel.
    for shoe, rail_elem in [
        ("upper_guide_shoe_0", "z_guide_0"),
        ("upper_guide_shoe_1", "z_guide_1"),
    ]:
        ctx.expect_gap(
            tool_plate,
            crosshead,
            axis="x",
            min_gap=0.004,
            positive_elem=shoe,
            negative_elem=rail_elem,
            name=f"{shoe} clears vertical rail",
        )
        ctx.expect_overlap(
            tool_plate,
            crosshead,
            axes="z",
            min_overlap=0.06,
            elem_a=shoe,
            elem_b=rail_elem,
            name=f"{shoe} is retained on vertical rail at home",
        )

    rest_crosshead = ctx.part_world_position(crosshead)
    with ctx.pose({y_slide: 0.60}):
        extended_crosshead = ctx.part_world_position(crosshead)
        ctx.expect_overlap(
            crosshead,
            frame,
            axes="y",
            min_overlap=0.18,
            elem_a="upper_bearing_body",
            elem_b="upper_y_rail",
            name="crosshead remains on Y rail at far travel",
        )
    ctx.check(
        "crosshead travels along positive Y",
        rest_crosshead is not None
        and extended_crosshead is not None
        and extended_crosshead[1] > rest_crosshead[1] + 0.55,
        details=f"home={rest_crosshead}, far={extended_crosshead}",
    )

    rest_tool = ctx.part_world_position(tool_plate)
    with ctx.pose({z_slide: 0.30}):
        lowered_tool = ctx.part_world_position(tool_plate)
        ctx.expect_overlap(
            tool_plate,
            crosshead,
            axes="z",
            min_overlap=0.06,
            elem_a="lower_guide_shoe_0",
            elem_b="z_guide_0",
            name="tool plate remains guided when lowered",
        )
    ctx.check(
        "tool plate descends on positive command",
        rest_tool is not None
        and lowered_tool is not None
        and lowered_tool[2] < rest_tool[2] - 0.28,
        details=f"home={rest_tool}, lowered={lowered_tool}",
    )

    return ctx.report()


object_model = build_object_model()
