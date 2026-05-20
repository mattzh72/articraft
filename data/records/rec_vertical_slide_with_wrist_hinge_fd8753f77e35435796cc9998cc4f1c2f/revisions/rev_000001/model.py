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


def _box(part, name, size, xyz, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _cylinder(part, name, radius, length, xyz, material, rpy=(0.0, 0.0, 0.0)):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_slide_wrist_study")

    model.material("painted_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    model.material("dark_oxide", rgba=(0.035, 0.038, 0.040, 1.0))
    model.material("machined_steel", rgba=(0.58, 0.60, 0.58, 1.0))
    model.material("bearing_black", rgba=(0.015, 0.016, 0.018, 1.0))
    model.material("bronze_bushing", rgba=(0.62, 0.42, 0.18, 1.0))
    model.material("blue_gray", rgba=(0.24, 0.31, 0.36, 1.0))
    model.material("rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    model.material("warning_stop", rgba=(0.55, 0.07, 0.04, 1.0))

    frame = model.part("frame")

    # Welded base and column structure.
    _box(frame, "base_plate", (0.56, 0.42, 0.035), (0.0, 0.0, 0.0175), "painted_steel")
    for ix, x in enumerate((-0.21, 0.21)):
        for iy, y in enumerate((-0.15, 0.15)):
            _cylinder(
                frame,
                f"leveling_foot_{ix}_{iy}",
                0.032,
                0.014,
                (x, y, 0.004),
                "rubber",
            )

    _box(frame, "column_web", (0.14, 0.10, 1.10), (0.0, 0.075, 0.585), "painted_steel")
    _box(frame, "column_flange_0", (0.026, 0.125, 1.10), (-0.083, 0.075, 0.585), "painted_steel")
    _box(frame, "column_flange_1", (0.026, 0.125, 1.10), (0.083, 0.075, 0.585), "painted_steel")
    _box(frame, "bottom_column_plate", (0.30, 0.18, 0.026), (0.0, 0.054, 0.048), "painted_steel")
    _box(frame, "top_column_plate", (0.30, 0.18, 0.026), (0.0, 0.054, 1.148), "painted_steel")
    _box(frame, "front_gusset_0", (0.026, 0.18, 0.032), (-0.109, -0.010, 0.077), "painted_steel")
    _box(frame, "front_gusset_1", (0.026, 0.18, 0.032), (0.109, -0.010, 0.077), "painted_steel")

    # Stationary vertical guides: two hardened round rails on standoff bars with bolted end blocks.
    for i, x in enumerate((-0.065, 0.065)):
        _box(frame, f"rail_standoff_{i}", (0.036, 0.054, 0.965), (x, -0.001, 0.595), "dark_oxide")
        if i == 0:
            frame.visual(
                Cylinder(radius=0.012, length=0.960),
                origin=Origin(xyz=(x, -0.035, 0.595)),
                material="machined_steel",
                name="guide_rail_0",
            )
        else:
            frame.visual(
                Cylinder(radius=0.012, length=0.960),
                origin=Origin(xyz=(x, -0.035, 0.595)),
                material="machined_steel",
                name="guide_rail_1",
            )
        _box(frame, f"lower_rail_clamp_{i}", (0.060, 0.066, 0.044), (x, -0.014, 0.095), "dark_oxide")
        _box(frame, f"upper_rail_clamp_{i}", (0.060, 0.066, 0.044), (x, -0.014, 1.095), "dark_oxide")

    _box(frame, "lower_stop_block", (0.145, 0.030, 0.028), (0.0, -0.055, 0.155), "warning_stop")
    _box(frame, "upper_stop_block", (0.145, 0.030, 0.028), (0.0, -0.055, 1.035), "warning_stop")

    # Removable front service cover on the column, with four socket-head screws.
    _box(frame, "column_cover", (0.070, 0.006, 0.455), (0.0, 0.022, 0.610), "dark_oxide")
    for ix, x in enumerate((-0.024, 0.024)):
        for iz, z in enumerate((0.425, 0.795)):
            _cylinder(
                frame,
                f"column_cover_screw_{ix}_{iz}",
                0.0055,
                0.006,
                (x, 0.018, z),
                "machined_steel",
                rpy=(-pi / 2.0, 0.0, 0.0),
            )

    carriage = model.part("carriage")

    # Moving slide plate and linear-bearing blocks.
    _box(carriage, "slide_plate", (0.230, 0.030, 0.250), (0.0, -0.085, 0.0), "blue_gray")
    _box(carriage, "top_crosshead", (0.215, 0.036, 0.032), (0.0, -0.084, 0.127), "blue_gray")
    _box(carriage, "bottom_crosshead", (0.215, 0.036, 0.032), (0.0, -0.084, -0.127), "blue_gray")
    for ix, x in enumerate((-0.065, 0.065)):
        for iz, z in enumerate((-0.070, 0.070)):
            if ix == 0 and iz == 0:
                carriage.visual(
                    Box((0.056, 0.036, 0.066)),
                    origin=Origin(xyz=(x, -0.065, z)),
                    material="bearing_black",
                    name="bearing_block_0_0",
                )
            elif ix == 0 and iz == 1:
                carriage.visual(
                    Box((0.056, 0.036, 0.066)),
                    origin=Origin(xyz=(x, -0.065, z)),
                    material="bearing_black",
                    name="bearing_block_0_1",
                )
            else:
                _box(
                    carriage,
                    f"bearing_block_{ix}_{iz}",
                    (0.056, 0.036, 0.066),
                    (x, -0.065, z),
                    "bearing_black",
                )
            _box(
                carriage,
                f"bearing_cap_{ix}_{iz}",
                (0.041, 0.006, 0.048),
                (x, -0.086, z),
                "machined_steel",
            )
            for sx in (-0.014, 0.014):
                _cylinder(
                    carriage,
                    f"bearing_cap_screw_{ix}_{iz}_{0 if sx < 0 else 1}",
                    0.004,
                    0.006,
                    (x + sx, -0.092, z),
                    "dark_oxide",
                    rpy=(-pi / 2.0, 0.0, 0.0),
                )

    # Removable access cover and front pivot mounting flange on the moving carriage.
    _box(carriage, "carriage_cover", (0.105, 0.006, 0.135), (0.0, -0.103, 0.0), "dark_oxide")
    for ix, x in enumerate((-0.040, 0.040)):
        for iz, z in enumerate((-0.050, 0.050)):
            _cylinder(
                carriage,
                f"carriage_cover_screw_{ix}_{iz}",
                0.0048,
                0.006,
                (x, -0.109, z),
                "machined_steel",
                rpy=(-pi / 2.0, 0.0, 0.0),
            )

    _box(carriage, "wrist_mount_plate", (0.185, 0.016, 0.145), (0.0, -0.108, 0.0), "blue_gray")
    carriage.visual(
        Box((0.030, 0.116, 0.104)),
        origin=Origin(xyz=(-0.075, -0.155, 0.0)),
        material="blue_gray",
        name="left_hinge_cheek",
    )
    carriage.visual(
        Box((0.030, 0.116, 0.104)),
        origin=Origin(xyz=(0.075, -0.155, 0.0)),
        material="blue_gray",
        name="right_hinge_cheek",
    )
    _box(carriage, "left_cheek_rib", (0.020, 0.104, 0.025), (-0.075, -0.132, -0.064), "blue_gray")
    _box(carriage, "right_cheek_rib", (0.020, 0.104, 0.025), (0.075, -0.132, -0.064), "blue_gray")
    _cylinder(
        carriage,
        "left_pivot_bushing",
        0.033,
        0.014,
        (-0.097, -0.175, 0.0),
        "bronze_bushing",
        rpy=(0.0, pi / 2.0, 0.0),
    )
    _cylinder(
        carriage,
        "right_pivot_bushing",
        0.033,
        0.014,
        (0.097, -0.175, 0.0),
        "bronze_bushing",
        rpy=(0.0, pi / 2.0, 0.0),
    )

    wrist = model.part("wrist")

    # Hinged wrist: central trunnion barrel, forward fabricated link, and bolted tool flange.
    wrist.visual(
        Cylinder(radius=0.025, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="machined_steel",
        name="pivot_barrel",
    )
    _box(wrist, "wrist_link", (0.070, 0.220, 0.046), (0.0, -0.105, 0.0), "painted_steel")
    _box(wrist, "upper_link_rib", (0.086, 0.160, 0.020), (0.0, -0.095, 0.033), "painted_steel")
    _box(wrist, "lower_link_rib", (0.086, 0.160, 0.020), (0.0, -0.095, -0.033), "painted_steel")
    _box(wrist, "tool_flange", (0.160, 0.018, 0.140), (0.0, -0.221, 0.0), "machined_steel")
    _cylinder(
        wrist,
        "tool_boss",
        0.027,
        0.018,
        (0.0, -0.239, 0.0),
        "machined_steel",
        rpy=(-pi / 2.0, 0.0, 0.0),
    )
    for ix, x in enumerate((-0.055, 0.055)):
        for iz, z in enumerate((-0.045, 0.045)):
            _cylinder(
                wrist,
                f"flange_bolt_{ix}_{iz}",
                0.0075,
                0.008,
                (x, -0.234, z),
                "dark_oxide",
                rpy=(-pi / 2.0, 0.0, 0.0),
            )
    _cylinder(
        wrist,
        "pivot_grease_fitting",
        0.005,
        0.026,
        (0.0, -0.020, 0.027),
        "bronze_bushing",
        rpy=(pi / 2.0, 0.0, 0.0),
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.22, lower=0.0, upper=0.450),
    )

    model.articulation(
        "carriage_to_wrist",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=wrist,
        origin=Origin(xyz=(0.0, -0.175, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.0, lower=-0.65, upper=0.85),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    wrist = object_model.get_part("wrist")
    slide_joint = object_model.get_articulation("frame_to_carriage")
    wrist_joint = object_model.get_articulation("carriage_to_wrist")

    ctx.check(
        "two explicit motion axes",
        len(object_model.articulations) == 2,
        details=f"articulations={len(object_model.articulations)}",
    )

    ctx.expect_contact(
        frame,
        carriage,
        elem_a="guide_rail_0",
        elem_b="bearing_block_0_0",
        contact_tol=0.0008,
        name="bearing shoe touches the guide rail",
    )
    ctx.expect_overlap(
        frame,
        carriage,
        axes="xz",
        min_overlap=0.020,
        elem_a="guide_rail_0",
        elem_b="bearing_block_0_0",
        name="bearing block is aligned to its vertical rail",
    )
    ctx.expect_contact(
        carriage,
        wrist,
        elem_a="right_hinge_cheek",
        elem_b="pivot_barrel",
        contact_tol=0.0008,
        name="right cheek supports wrist trunnion",
    )
    ctx.expect_contact(
        carriage,
        wrist,
        elem_a="left_hinge_cheek",
        elem_b="pivot_barrel",
        contact_tol=0.0008,
        name="left cheek supports wrist trunnion",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide_joint: 0.45}):
        high_pos = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            frame,
            carriage,
            axes="z",
            min_overlap=0.035,
            elem_a="guide_rail_0",
            elem_b="bearing_block_0_1",
            name="raised carriage still remains on guide rail",
        )
    ctx.check(
        "carriage translates upward on the column",
        rest_pos is not None and high_pos is not None and high_pos[2] > rest_pos[2] + 0.40,
        details=f"rest={rest_pos}, raised={high_pos}",
    )

    neutral_flange = ctx.part_element_world_aabb(wrist, elem="tool_flange")
    with ctx.pose({wrist_joint: -0.65}):
        raised_flange = ctx.part_element_world_aabb(wrist, elem="tool_flange")
    if neutral_flange is not None and raised_flange is not None:
        neutral_z = 0.5 * (neutral_flange[0][2] + neutral_flange[1][2])
        raised_z = 0.5 * (raised_flange[0][2] + raised_flange[1][2])
    else:
        neutral_z = raised_z = None
    ctx.check(
        "wrist hinge pitches the flange upward",
        neutral_z is not None and raised_z is not None and raised_z > neutral_z + 0.10,
        details=f"neutral_z={neutral_z}, raised_z={raised_z}",
    )

    return ctx.report()


object_model = build_object_model()
