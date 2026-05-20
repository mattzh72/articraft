from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _build_blade_mesh(
    mesh_name: str,
    *,
    length: float,
    height: float,
    thickness: float,
):
    profile = [
        (0.0, 0.0),
        (length * 0.42, 0.0),
        (length * 0.60, height * 0.30),
        (length, height * 0.80),
        (length * 0.78, height),
        (0.002, height),
    ]
    geom = ExtrudeGeometry.from_z0(profile, thickness)
    geom.rotate_x(math.pi / 2.0)
    geom.translate(0.0, thickness * 0.5, 0.0)
    return mesh_from_geometry(geom, mesh_name)


def _build_slider_button_mesh(mesh_name: str):
    geom = ExtrudeGeometry.from_z0(rounded_rect_profile(0.016, 0.010, 0.0022), 0.005)
    return mesh_from_geometry(geom, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_utility_knife")

    shell_yellow = model.material("shell_yellow", rgba=(0.84, 0.74, 0.18, 1.0))
    shell_black = model.material("shell_black", rgba=(0.12, 0.12, 0.13, 1.0))
    steel = model.material("steel", rgba=(0.67, 0.69, 0.72, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.86, 0.88, 0.90, 1.0))

    main_blade_mesh = _build_blade_mesh(
        "utility_knife_main_blade",
        length=0.040,
        height=0.0155,
        thickness=0.0008,
    )
    spare_blade_mesh = _build_blade_mesh(
        "utility_knife_spare_blade",
        length=0.028,
        height=0.0115,
        thickness=0.0006,
    )
    slider_button_mesh = _build_slider_button_mesh("utility_knife_slider_button")

    handle_shell = model.part("handle_shell")
    handle_shell.visual(
        Box((0.152, 0.026, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=shell_yellow,
        name="base_plate",
    )
    handle_shell.visual(
        Box((0.146, 0.0025, 0.020)),
        origin=Origin(xyz=(-0.002, -0.01175, 0.014)),
        material=shell_yellow,
        name="left_shell_wall",
    )
    handle_shell.visual(
        Box((0.078, 0.0025, 0.020)),
        origin=Origin(xyz=(0.032, 0.01125, 0.014)),
        material=shell_yellow,
        name="right_shell_front",
    )
    handle_shell.visual(
        Box((0.020, 0.0025, 0.020)),
        origin=Origin(xyz=(-0.066, 0.01125, 0.014)),
        material=shell_yellow,
        name="right_shell_rear",
    )
    handle_shell.visual(
        Box((0.050, 0.0025, 0.004)),
        origin=Origin(xyz=(-0.031, 0.01125, 0.008)),
        material=shell_yellow,
        name="door_frame_lower",
    )
    handle_shell.visual(
        Box((0.050, 0.0025, 0.004)),
        origin=Origin(xyz=(-0.031, 0.01125, 0.026)),
        material=shell_yellow,
        name="door_frame_upper",
    )
    handle_shell.visual(
        Box((0.004, 0.0025, 0.016)),
        origin=Origin(xyz=(-0.006, 0.01125, 0.016)),
        material=shell_yellow,
        name="door_frame_front",
    )
    handle_shell.visual(
        Box((0.004, 0.0025, 0.016)),
        origin=Origin(xyz=(-0.056, 0.01125, 0.016)),
        material=shell_yellow,
        name="door_hinge_post",
    )
    handle_shell.visual(
        Box((0.128, 0.022, 0.002)),
        origin=Origin(xyz=(0.015, 0.0, 0.011)),
        material=dark_steel,
        name="channel_floor",
    )
    handle_shell.visual(
        Box((0.114, 0.0085, 0.004)),
        origin=Origin(xyz=(0.010, -0.00675, 0.028)),
        material=shell_black,
        name="top_left_rail",
    )
    handle_shell.visual(
        Box((0.114, 0.0085, 0.004)),
        origin=Origin(xyz=(0.010, 0.00675, 0.028)),
        material=shell_black,
        name="top_right_rail",
    )
    handle_shell.visual(
        Box((0.030, 0.022, 0.004)),
        origin=Origin(xyz=(-0.060, 0.0, 0.028)),
        material=shell_black,
        name="rear_cap",
    )
    handle_shell.visual(
        Box((0.024, 0.022, 0.004)),
        origin=Origin(xyz=(0.086, 0.0, 0.002)),
        material=dark_steel,
        name="nose_bottom",
    )
    handle_shell.visual(
        Box((0.024, 0.0055, 0.016)),
        origin=Origin(xyz=(0.086, -0.00775, 0.014)),
        material=dark_steel,
        name="nose_left_cheek",
    )
    handle_shell.visual(
        Box((0.024, 0.0055, 0.016)),
        origin=Origin(xyz=(0.086, 0.00775, 0.014)),
        material=dark_steel,
        name="nose_right_cheek",
    )
    handle_shell.visual(
        Box((0.070, 0.0014, 0.011)),
        origin=Origin(xyz=(-0.020, -0.01295, 0.015)),
        material=shell_black,
        name="left_grip_inlay",
    )
    handle_shell.visual(
        Box((0.044, 0.0014, 0.010)),
        origin=Origin(xyz=(0.024, 0.01295, 0.015)),
        material=shell_black,
        name="right_grip_inlay",
    )
    handle_shell.visual(
        Box((0.010, 0.022, 0.018)),
        origin=Origin(xyz=(-0.073, 0.0, 0.013)),
        material=shell_black,
        name="rear_bumper",
    )
    handle_shell.visual(
        Box((0.024, 0.014, 0.002)),
        origin=Origin(xyz=(-0.060, 0.0, 0.031)),
        material=shell_black,
        name="rear_spine_pad",
    )
    handle_shell.visual(
        Box((0.046, 0.0025, 0.014)),
        origin=Origin(xyz=(-0.031, 0.00125, 0.015)),
        material=dark_steel,
        name="cavity_inner_wall",
    )
    handle_shell.visual(
        Box((0.046, 0.0105, 0.002)),
        origin=Origin(xyz=(-0.031, 0.00575, 0.009)),
        material=dark_steel,
        name="cavity_floor",
    )
    handle_shell.visual(
        Box((0.046, 0.0105, 0.002)),
        origin=Origin(xyz=(-0.031, 0.00575, 0.021)),
        material=dark_steel,
        name="cavity_ceiling",
    )
    handle_shell.visual(
        Box((0.0025, 0.0105, 0.014)),
        origin=Origin(xyz=(-0.00775, 0.00575, 0.015)),
        material=dark_steel,
        name="cavity_front_wall",
    )
    handle_shell.visual(
        Box((0.0025, 0.0105, 0.014)),
        origin=Origin(xyz=(-0.05425, 0.00575, 0.015)),
        material=dark_steel,
        name="cavity_rear_wall",
    )
    handle_shell.visual(
        spare_blade_mesh,
        origin=Origin(xyz=(-0.047, 0.0050, 0.0102)),
        material=blade_steel,
        name="spare_blade_a",
    )
    handle_shell.visual(
        spare_blade_mesh,
        origin=Origin(xyz=(-0.040, 0.0068, 0.0102)),
        material=blade_steel,
        name="spare_blade_b",
    )
    handle_shell.inertial = Inertial.from_geometry(
        Box((0.174, 0.028, 0.032)),
        mass=0.32,
        origin=Origin(xyz=(0.004, 0.0, 0.016)),
    )

    blade_carrier = model.part("blade_carrier")
    blade_carrier.visual(
        Box((0.052, 0.0096, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_steel,
        name="carrier_body",
    )
    blade_carrier.visual(
        Box((0.014, 0.008, 0.008)),
        origin=Origin(xyz=(0.025, 0.0, 0.016)),
        material=steel,
        name="carrier_clamp",
    )
    blade_carrier.visual(
        Box((0.010, 0.003, 0.012)),
        origin=Origin(xyz=(-0.004, 0.0, 0.024)),
        material=steel,
        name="slider_stem",
    )
    blade_carrier.visual(
        slider_button_mesh,
        origin=Origin(xyz=(-0.004, 0.0, 0.0300)),
        material=shell_black,
        name="slider_button",
    )
    blade_carrier.visual(
        Box((0.012, 0.0012, 0.0012)),
        origin=Origin(xyz=(-0.004, 0.0, 0.0354)),
        material=shell_black,
        name="slider_ridge",
    )
    blade_carrier.visual(
        main_blade_mesh,
        origin=Origin(xyz=(0.022, 0.0, 0.0135)),
        material=blade_steel,
        name="blade_edge",
    )
    blade_carrier.inertial = Inertial.from_geometry(
        Box((0.070, 0.010, 0.025)),
        mass=0.09,
        origin=Origin(xyz=(0.010, 0.0, 0.018)),
    )

    storage_door = model.part("storage_door")
    storage_door.visual(
        Box((0.048, 0.002, 0.016)),
        origin=Origin(xyz=(0.024, 0.0, 0.0)),
        material=shell_yellow,
        name="door_panel",
    )
    storage_door.visual(
        Box((0.016, 0.0012, 0.004)),
        origin=Origin(xyz=(0.032, 0.0016, 0.0)),
        material=shell_black,
        name="door_pull",
    )
    storage_door.inertial = Inertial.from_geometry(
        Box((0.050, 0.004, 0.018)),
        mass=0.03,
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
    )

    model.articulation(
        "blade_slide",
        ArticulationType.PRISMATIC,
        parent=handle_shell,
        child=blade_carrier,
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.20,
            lower=0.0,
            upper=0.042,
        ),
    )
    model.articulation(
        "storage_door_hinge",
        ArticulationType.REVOLUTE,
        parent=handle_shell,
        child=storage_door,
        origin=Origin(xyz=(-0.056, 0.0135, 0.016)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    handle_shell = object_model.get_part("handle_shell")
    blade_carrier = object_model.get_part("blade_carrier")
    storage_door = object_model.get_part("storage_door")
    blade_slide = object_model.get_articulation("blade_slide")
    storage_door_hinge = object_model.get_articulation("storage_door_hinge")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "blade slide axis",
        tuple(blade_slide.axis) == (1.0, 0.0, 0.0),
        f"Expected blade_slide axis (1, 0, 0), got {blade_slide.axis!r}.",
    )
    ctx.check(
        "storage door hinge axis",
        tuple(storage_door_hinge.axis) == (0.0, 0.0, 1.0),
        f"Expected storage_door_hinge axis (0, 0, 1), got {storage_door_hinge.axis!r}.",
    )

    ctx.expect_gap(
        blade_carrier,
        handle_shell,
        axis="z",
        positive_elem="carrier_body",
        negative_elem="channel_floor",
        max_gap=0.0002,
        max_penetration=0.0,
        name="carrier rides on channel floor",
    )
    ctx.expect_gap(
        storage_door,
        handle_shell,
        axis="y",
        positive_elem="door_panel",
        negative_elem="door_frame_lower",
        max_gap=0.0002,
        max_penetration=0.0,
        name="storage door sits flush with shell opening",
    )

    handle_aabb = ctx.part_world_aabb(handle_shell)
    blade_rest_aabb = ctx.part_element_world_aabb(blade_carrier, elem="blade_edge")
    carrier_rest_pos = ctx.part_world_position(blade_carrier)
    door_pull_rest_aabb = ctx.part_element_world_aabb(storage_door, elem="door_pull")

    if (
        handle_aabb is None
        or blade_rest_aabb is None
        or carrier_rest_pos is None
        or door_pull_rest_aabb is None
    ):
        ctx.fail("utility knife test probes ready", "Missing world-space test measurements.")
        return ctx.report()

    handle_front_x = handle_aabb[1][0]
    ctx.check(
        "blade retracts inside shell nose",
        blade_rest_aabb[1][0] < handle_front_x - 0.020,
        (
            f"Retracted blade tip x={blade_rest_aabb[1][0]:.4f} should stay well behind "
            f"shell nose x={handle_front_x:.4f}."
        ),
    )

    with ctx.pose({blade_slide: 0.042}):
        carrier_extended_pos = ctx.part_world_position(blade_carrier)
        blade_extended_aabb = ctx.part_element_world_aabb(blade_carrier, elem="blade_edge")
        if carrier_extended_pos is None or blade_extended_aabb is None:
            ctx.fail(
                "blade extended pose measurable",
                "Missing carrier position or blade AABB in the extended pose.",
            )
        else:
            ctx.check(
                "blade carrier slides forward",
                carrier_extended_pos[0] > carrier_rest_pos[0] + 0.040,
                (
                    f"Carrier x moved from {carrier_rest_pos[0]:.4f} to "
                    f"{carrier_extended_pos[0]:.4f}, expected > 0.040 m forward travel."
                ),
            )
            ctx.check(
                "blade extends beyond shell nose",
                blade_extended_aabb[1][0] > handle_front_x + 0.012,
                (
                    f"Extended blade tip x={blade_extended_aabb[1][0]:.4f} should exceed "
                    f"shell nose x={handle_front_x:.4f}."
                ),
            )
            ctx.expect_gap(
                blade_carrier,
                handle_shell,
                axis="z",
                positive_elem="carrier_body",
                negative_elem="channel_floor",
                max_gap=0.0002,
                max_penetration=0.0,
                name="carrier remains guided at full extension",
            )

    with ctx.pose({storage_door_hinge: 1.20}):
        door_pull_open_aabb = ctx.part_element_world_aabb(storage_door, elem="door_pull")
        if door_pull_open_aabb is None:
            ctx.fail(
                "door open pose measurable",
                "Missing door-pull AABB in the open pose.",
            )
        else:
            ctx.check(
                "storage door swings outward",
                door_pull_open_aabb[1][1] > door_pull_rest_aabb[1][1] + 0.020,
                (
                    f"Door pull max y moved from {door_pull_rest_aabb[1][1]:.4f} to "
                    f"{door_pull_open_aabb[1][1]:.4f}, expected > 0.020 m outward swing."
                ),
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
