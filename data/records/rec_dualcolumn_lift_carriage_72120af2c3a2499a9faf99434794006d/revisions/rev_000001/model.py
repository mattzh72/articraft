from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def _box(part, name: str, size, xyz, material: str, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _cyl(part, name: str, radius: float, length: float, xyz, material: str, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_column_lift_carriage_study")

    model.material("painted_frame", rgba=(0.18, 0.20, 0.21, 1.0))
    model.material("machined_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    model.material("dark_oxide", rgba=(0.04, 0.045, 0.045, 1.0))
    model.material("bronze_liner", rgba=(0.78, 0.52, 0.22, 1.0))
    model.material("rubber_stop", rgba=(0.015, 0.015, 0.014, 1.0))
    model.material("cover_plate", rgba=(0.25, 0.28, 0.29, 1.0))

    frame = model.part("frame")

    # Welded/fabricated fixed structure: two columns tied by crossmembers.
    _box(frame, "bottom_crossmember", (1.16, 0.18, 0.12), (0.0, 0.0, 0.10), "painted_frame")
    _box(frame, "top_crossmember", (1.16, 0.18, 0.12), (0.0, 0.0, 1.70), "painted_frame")
    for side, x in (("left", -0.48), ("right", 0.48)):
        _box(frame, f"{side}_column", (0.10, 0.10, 1.50), (x, 0.0, 0.90), "painted_frame")
        _box(frame, f"{side}_foot_plate", (0.24, 0.24, 0.025), (x, 0.0, 0.0225), "painted_frame")
        _box(frame, f"{side}_top_cap", (0.20, 0.18, 0.035), (x, 0.0, 1.7775), "painted_frame")
        _box(frame, f"{side}_rail", (0.056, 0.022, 1.30), (x, -0.060, 0.90), "machined_steel")
        _box(frame, f"{side}_rail_backer", (0.080, 0.018, 1.34), (x, -0.047, 0.90), "painted_frame")
        _box(frame, f"{side}_lower_gusset", (0.18, 0.040, 0.050), (x * 0.88, -0.055, 0.255), "painted_frame", rpy=(0.0, 0.72 if x < 0 else -0.72, 0.0))
        _box(frame, f"{side}_upper_gusset", (0.18, 0.040, 0.050), (x * 0.88, -0.055, 1.545), "painted_frame", rpy=(0.0, -0.72 if x < 0 else 0.72, 0.0))

        # Rail and base bolt patterns; the heads are slightly seated into their plates.
        for z in (0.36, 0.62, 0.90, 1.18, 1.44):
            _cyl(frame, f"{side}_rail_bolt_{int(z * 100)}", 0.010, 0.006, (x, -0.045, z), "dark_oxide", rpy=(pi / 2, 0.0, 0.0))
        for bx, by in ((-0.065, -0.065), (-0.065, 0.065), (0.065, -0.065), (0.065, 0.065)):
            _cyl(frame, f"{side}_anchor_bolt_{bx}_{by}", 0.012, 0.012, (x + bx, by, 0.041), "dark_oxide")

    # Bearing pedestals for the vertical screw and the synchronizing cross shaft.
    _box(frame, "lower_screw_bearing", (0.18, 0.12, 0.080), (0.0, -0.055, 0.220), "painted_frame")
    _box(frame, "upper_screw_bearing", (0.18, 0.12, 0.080), (0.0, -0.055, 1.580), "painted_frame")
    _box(frame, "lower_bearing_web", (0.24, 0.10, 0.060), (0.0, -0.050, 0.170), "painted_frame")
    _box(frame, "upper_bearing_web", (0.24, 0.10, 0.060), (0.0, -0.050, 1.630), "painted_frame")

    for side, x in (("left", -0.42), ("right", 0.42)):
        _box(frame, f"{side}_shaft_saddle", (0.22, 0.16, 0.060), (x, -0.115, 1.485), "painted_frame")
        _box(frame, f"{side}_shaft_bearing", (0.16, 0.12, 0.110), (x, -0.180, 1.550), "painted_frame")
        _cyl(frame, f"{side}_bearing_face", 0.041, 0.012, (x, -0.246, 1.550), "machined_steel", rpy=(pi / 2, 0.0, 0.0))
        for zoff in (-0.034, 0.034):
            _cyl(frame, f"{side}_bearing_cap_bolt_{zoff}", 0.008, 0.010, (x, -0.253, 1.550 + zoff), "dark_oxide", rpy=(pi / 2, 0.0, 0.0))

    # Limit hardware: elastomer pads on machined stop posts.
    for idx, x in enumerate((-0.20, 0.20)):
        _box(frame, f"lower_stop_post_{idx}", (0.075, 0.050, 0.130), (x, -0.090, 0.225), "painted_frame")
        _box(frame, f"lower_stop_pad_{idx}", (0.085, 0.055, 0.040), (x, -0.090, 0.285), "rubber_stop")
        _box(frame, f"upper_stop_post_{idx}", (0.075, 0.050, 0.060), (x, -0.090, 1.610), "painted_frame")
        _box(frame, f"upper_stop_pad_{idx}", (0.085, 0.055, 0.040), (x, -0.090, 1.565), "rubber_stop")

    # Fixed cable/chain guide trays that reveal the lift-drive path without hiding the mechanism.
    for side, x in (("left", -0.34), ("right", 0.34)):
        _box(frame, f"{side}_chain_guard_back", (0.045, 0.020, 1.22), (x, -0.210, 0.92), "dark_oxide")
        _box(frame, f"{side}_chain_guard_lip", (0.065, 0.018, 0.030), (x, -0.210, 0.32), "dark_oxide")
        _box(frame, f"{side}_chain_guard_top_lip", (0.065, 0.018, 0.030), (x, -0.210, 1.52), "dark_oxide")

    carriage = model.part("carriage")
    _box(carriage, "main_plate", (0.64, 0.050, 0.400), (0.0, -0.130, 0.0), "painted_frame")
    _box(carriage, "upper_tie_plate", (0.70, 0.060, 0.055), (0.0, -0.150, 0.205), "painted_frame")
    _box(carriage, "lower_tie_plate", (0.70, 0.060, 0.055), (0.0, -0.150, -0.205), "painted_frame")
    _box(carriage, "left_side_plate", (0.105, 0.060, 0.450), (-0.310, -0.145, 0.0), "painted_frame")
    _box(carriage, "right_side_plate", (0.105, 0.060, 0.450), (0.310, -0.145, 0.0), "painted_frame")

    for side, sx in (("left", -1.0), ("right", 1.0)):
        x = sx * 0.48
        arm_x = sx * 0.385
        for level, z in (("lower", -0.135), ("upper", 0.135)):
            _box(carriage, f"{side}_{level}_outrigger", (0.225, 0.080, 0.065), (arm_x, -0.115, z), "painted_frame")
            _box(carriage, f"{side}_{level}_guide_block", (0.160, 0.044, 0.115), (x, -0.088, z), "painted_frame")
            _box(carriage, f"{side}_{level}_liner", (0.118, 0.012, 0.084), (x, -0.081, z), "bronze_liner")
            _cyl(carriage, f"{side}_{level}_guide_bolt_top", 0.008, 0.010, (x - sx * 0.040, -0.114, z + 0.032), "dark_oxide", rpy=(pi / 2, 0.0, 0.0))
            _cyl(carriage, f"{side}_{level}_guide_bolt_bottom", 0.008, 0.010, (x + sx * 0.040, -0.114, z - 0.032), "dark_oxide", rpy=(pi / 2, 0.0, 0.0))

    # Split drive nut around the exposed screw, with a yoke back to the carriage plate.
    _box(carriage, "nut_bridge", (0.205, 0.045, 0.100), (0.0, -0.100, 0.0), "painted_frame")
    _box(carriage, "left_nut_half", (0.060, 0.085, 0.135), (-0.052, -0.055, 0.0), "bronze_liner")
    _box(carriage, "right_nut_half", (0.060, 0.085, 0.135), (0.052, -0.055, 0.0), "bronze_liner")
    for z in (-0.043, 0.043):
        _cyl(carriage, f"nut_clamp_bolt_{z}", 0.007, 0.135, (0.0, -0.100, z), "dark_oxide", rpy=(0.0, pi / 2, 0.0))

    for idx, x in enumerate((-0.20, 0.20)):
        _box(carriage, f"lower_striker_{idx}", (0.080, 0.050, 0.040), (x, -0.120, -0.225), "machined_steel")
        _box(carriage, f"upper_striker_{idx}", (0.080, 0.050, 0.040), (x, -0.120, 0.225), "machined_steel")

    for row, z in enumerate((-0.150, 0.0, 0.150)):
        for col, x in enumerate((-0.205, -0.105, 0.105, 0.205)):
            _cyl(carriage, f"plate_bolt_{row}_{col}", 0.010, 0.010, (x, -0.160, z), "dark_oxide", rpy=(pi / 2, 0.0, 0.0))

    lead_screw = model.part("lead_screw")
    _cyl(lead_screw, "screw_core", 0.016, 1.34, (0.0, 0.0, 0.0), "machined_steel")
    for i, z in enumerate([(-0.58 + 0.058 * n) for n in range(21)]):
        _cyl(lead_screw, f"thread_crest_{i}", 0.020, 0.010, (0.0, 0.0, z), "machined_steel")
    _cyl(lead_screw, "lower_locknut", 0.030, 0.035, (0.0, 0.0, -0.590), "dark_oxide")
    _cyl(lead_screw, "upper_locknut", 0.030, 0.035, (0.0, 0.0, 0.590), "dark_oxide")

    sync_shaft = model.part("sync_shaft")
    _cyl(sync_shaft, "shaft_core", 0.018, 0.94, (0.0, 0.0, 0.0), "machined_steel", rpy=(0.0, pi / 2, 0.0))
    for side, x in (("left", -0.250), ("right", 0.250)):
        _cyl(sync_shaft, f"{side}_timing_pulley", 0.060, 0.060, (x, 0.0, 0.0), "dark_oxide", rpy=(0.0, pi / 2, 0.0))
        _cyl(sync_shaft, f"{side}_pulley_flange_a", 0.066, 0.008, (x - 0.034, 0.0, 0.0), "machined_steel", rpy=(0.0, pi / 2, 0.0))
        _cyl(sync_shaft, f"{side}_pulley_flange_b", 0.066, 0.008, (x + 0.034, 0.0, 0.0), "machined_steel", rpy=(0.0, pi / 2, 0.0))
    _cyl(sync_shaft, "center_coupler", 0.026, 0.110, (0.0, 0.0, 0.0), "dark_oxide", rpy=(0.0, pi / 2, 0.0))

    access_cover = model.part("access_cover")
    _box(access_cover, "cover_panel", (0.300, 0.014, 0.225), (0.0, 0.0, 0.0), "cover_plate")
    _box(access_cover, "cover_top_flange", (0.315, 0.026, 0.026), (0.0, -0.006, 0.112), "cover_plate")
    _box(access_cover, "cover_bottom_flange", (0.315, 0.026, 0.026), (0.0, -0.006, -0.112), "cover_plate")
    _box(access_cover, "pull_tab", (0.090, 0.025, 0.036), (0.0, -0.0195, 0.0), "dark_oxide")
    for idx, (x, z) in enumerate(((-0.120, -0.080), (-0.120, 0.080), (0.120, -0.080), (0.120, 0.080))):
        _box(access_cover, f"standoff_{idx}", (0.024, 0.006, 0.024), (x, 0.010, z), "machined_steel")
        _cyl(access_cover, f"captive_screw_{idx}", 0.009, 0.010, (x, -0.012, z), "dark_oxide", rpy=(pi / 2, 0.0, 0.0))

    model.articulation(
        "lead_screw_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=lead_screw,
        origin=Origin(xyz=(0.0, -0.055, 0.900)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=6.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )
    model.articulation(
        "sync_shaft_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=sync_shaft,
        origin=Origin(xyz=(0.0, -0.180, 1.550)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=8.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )
    model.articulation(
        "carriage_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.550)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.20, lower=0.0, upper=0.75),
        motion_properties=MotionProperties(damping=6.0, friction=4.0),
    )
    model.articulation(
        "cover_slide",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=access_cover,
        origin=Origin(xyz=(0.0, -0.168, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.12, lower=0.0, upper=0.08),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    lead_screw = object_model.get_part("lead_screw")
    sync_shaft = object_model.get_part("sync_shaft")
    access_cover = object_model.get_part("access_cover")
    slide = object_model.get_articulation("carriage_slide")
    cover_slide = object_model.get_articulation("cover_slide")

    # Captured bearing interfaces are represented by solid shaft proxies passing
    # through solid bearing blocks; the overlap is local and intentional.
    for bearing in ("lower_screw_bearing", "upper_screw_bearing"):
        ctx.allow_overlap(
            frame,
            lead_screw,
            elem_a=bearing,
            elem_b="screw_core",
            reason="The lead screw core is intentionally captured inside the bearing block proxy.",
        )
        ctx.expect_within(
            lead_screw,
            frame,
            axes="xy",
            inner_elem="screw_core",
            outer_elem=bearing,
            margin=0.0,
            name=f"{bearing} centers the lead screw",
        )

    for bearing in ("left_shaft_bearing", "right_shaft_bearing"):
        ctx.allow_overlap(
            frame,
            sync_shaft,
            elem_a=bearing,
            elem_b="shaft_core",
            reason="The synchronizing shaft is intentionally represented as a captured shaft inside the bearing housing proxy.",
        )
        ctx.expect_within(
            sync_shaft,
            frame,
            axes="yz",
            inner_elem="shaft_core",
            outer_elem=bearing,
            margin=0.0,
            name=f"{bearing} contains the cross-shaft diameter",
        )

    ctx.expect_gap(
        frame,
        carriage,
        axis="y",
        min_gap=0.002,
        max_gap=0.010,
        positive_elem="left_rail",
        negative_elem="left_upper_liner",
        name="left upper guide liner has running clearance",
    )
    ctx.expect_overlap(
        carriage,
        frame,
        axes="xz",
        min_overlap=0.045,
        elem_a="right_lower_liner",
        elem_b="right_rail",
        name="right lower guide liner tracks on the guide rail projection",
    )
    ctx.expect_contact(
        carriage,
        frame,
        elem_a="lower_striker_0",
        elem_b="lower_stop_pad_0",
        name="lower hard stop is seated at the lower travel limit",
    )
    ctx.expect_contact(
        access_cover,
        carriage,
        elem_a="standoff_0",
        elem_b="main_plate",
        name="access cover standoff seats on the carriage plate",
    )

    rest_pos = ctx.part_world_position(carriage)
    cover_rest = ctx.part_world_position(access_cover)
    with ctx.pose({slide: 0.75}):
        raised_pos = ctx.part_world_position(carriage)
        ctx.expect_contact(
            carriage,
            frame,
            elem_a="upper_striker_0",
            elem_b="upper_stop_pad_0",
            name="upper hard stop is seated at the upper travel limit",
        )
        ctx.expect_overlap(
            carriage,
            frame,
            axes="xz",
            min_overlap=0.045,
            elem_a="left_upper_liner",
            elem_b="left_rail",
            name="raised carriage remains engaged with the left guide rail",
        )

    with ctx.pose({cover_slide: 0.08}):
        cover_out = ctx.part_world_position(access_cover)

    ctx.check(
        "carriage travels upward on the guide columns",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.70,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )
    ctx.check(
        "access cover slides forward for removal",
        cover_rest is not None and cover_out is not None and cover_out[1] < cover_rest[1] - 0.07,
        details=f"rest={cover_rest}, extended={cover_out}",
    )

    return ctx.report()


object_model = build_object_model()
