from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, segments: int = 32) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * i) / segments),
            radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def _rear_shell_mesh(
    *,
    body_width: float,
    body_depth: float,
    body_length: float,
    opening_width: float,
    opening_length: float,
    opening_center_z: float,
):
    outer = rounded_rect_profile(body_width, body_length, radius=0.010, corner_segments=10)
    opening_shift = opening_center_z - (body_length / 2.0)
    opening = [
        (x, y + opening_shift)
        for x, y in rounded_rect_profile(
            opening_width,
            opening_length,
            radius=0.005,
            corner_segments=8,
        )
    ]
    shell = ExtrudeWithHolesGeometry(
        outer,
        [list(reversed(opening))],
        body_depth,
        cap=True,
        center=True,
        closed=True,
    )
    shell.rotate_x(math.pi / 2.0)
    shell.translate(0.0, 0.0, body_length / 2.0)
    return shell


def _antenna_sleeve_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
):
    sleeve = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments=40),
        [list(reversed(_circle_profile(inner_radius, segments=40)))],
        length,
        cap=True,
        center=True,
        closed=True,
    )
    sleeve.translate(0.0, 0.0, length / 2.0)
    return sleeve


def _add_button_pad(
    body,
    *,
    x: float,
    z: float,
    radius: float,
    thickness: float,
    material,
    name: str,
):
    body.visual(
        Cylinder(radius=radius, length=thickness),
        origin=Origin(
            xyz=(x, 0.0126, z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=material,
        name=name,
    )


def _add_button_bar(
    body,
    *,
    x: float,
    z: float,
    size_x: float,
    size_z: float,
    thickness: float,
    material,
    name: str,
):
    body.visual(
        Box((size_x, thickness, size_z)),
        origin=Origin(xyz=(x, 0.0125, z)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="universal_remote")

    body_width = 0.056
    body_depth = 0.024
    body_length = 0.188
    rear_opening_width = 0.044
    rear_opening_length = 0.108
    rear_opening_center_z = 0.076

    shell_black = model.material("shell_black", rgba=(0.12, 0.13, 0.15, 1.0))
    fascia_black = model.material("fascia_black", rgba=(0.18, 0.19, 0.21, 1.0))
    rubber_gray = model.material("rubber_gray", rgba=(0.33, 0.34, 0.36, 1.0))
    rubber_dark = model.material("rubber_dark", rgba=(0.14, 0.15, 0.17, 1.0))
    accent_red = model.material("accent_red", rgba=(0.70, 0.12, 0.12, 1.0))
    smoked_window = model.material("smoked_window", rgba=(0.08, 0.08, 0.10, 1.0))
    battery_door_gray = model.material("battery_door_gray", rgba=(0.22, 0.23, 0.25, 1.0))
    metal = model.material("metal", rgba=(0.72, 0.74, 0.77, 1.0))
    ferrule = model.material("ferrule", rgba=(0.60, 0.62, 0.66, 1.0))

    rear_shell_mesh = mesh_from_geometry(
        _rear_shell_mesh(
            body_width=body_width,
            body_depth=body_depth,
            body_length=body_length,
            opening_width=rear_opening_width,
            opening_length=rear_opening_length,
            opening_center_z=rear_opening_center_z,
        ),
        "remote_rear_shell",
    )
    antenna_sleeve_mesh = mesh_from_geometry(
        _antenna_sleeve_mesh(
            outer_radius=0.0042,
            inner_radius=0.0031,
            length=0.050,
        ),
        "remote_antenna_sleeve",
    )

    body = model.part("body")
    body.visual(
        rear_shell_mesh,
        material=shell_black,
        name="rear_shell",
    )
    body.visual(
        Box((0.052, 0.0030, 0.184)),
        origin=Origin(xyz=(0.0, 0.0105, 0.094)),
        material=fascia_black,
        name="front_fascia",
    )
    body.visual(
        Box((0.018, 0.0020, 0.010)),
        origin=Origin(xyz=(0.0, 0.0111, 0.177)),
        material=smoked_window,
        name="ir_window",
    )
    body.visual(
        antenna_sleeve_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.187)),
        material=metal,
        name="antenna_sleeve",
    )
    body.visual(
        Box((0.003, 0.010, 0.012)),
        origin=Origin(xyz=(-0.0055, 0.0, 0.192)),
        material=ferrule,
        name="antenna_left_brace",
    )
    body.visual(
        Box((0.003, 0.010, 0.012)),
        origin=Origin(xyz=(0.0055, 0.0, 0.192)),
        material=ferrule,
        name="antenna_right_brace",
    )

    _add_button_bar(
        body,
        x=0.0,
        z=0.162,
        size_x=0.018,
        size_z=0.010,
        thickness=0.0030,
        material=accent_red,
        name="power_button",
    )

    for index, x in enumerate((-0.015, 0.015)):
        _add_button_pad(
            body,
            x=x,
            z=0.142,
            radius=0.0044,
            thickness=0.0028,
            material=rubber_gray,
            name=f"top_button_{index}",
        )
        _add_button_pad(
            body,
            x=x,
            z=0.128,
            radius=0.0044,
            thickness=0.0028,
            material=rubber_gray,
            name=f"mid_button_{index}",
        )

    _add_button_pad(
        body,
        x=0.0,
        z=0.104,
        radius=0.013,
        thickness=0.0028,
        material=rubber_gray,
        name="nav_ring",
    )
    _add_button_pad(
        body,
        x=0.0,
        z=0.104,
        radius=0.005,
        thickness=0.0032,
        material=rubber_dark,
        name="ok_button",
    )

    _add_button_bar(
        body,
        x=-0.015,
        z=0.079,
        size_x=0.010,
        size_z=0.030,
        thickness=0.0028,
        material=rubber_gray,
        name="volume_rocker",
    )
    _add_button_bar(
        body,
        x=0.015,
        z=0.079,
        size_x=0.010,
        size_z=0.030,
        thickness=0.0028,
        material=rubber_gray,
        name="channel_rocker",
    )

    keypad_x = (-0.016, 0.0, 0.016)
    keypad_z = (0.056, 0.041, 0.026, 0.011)
    for row, z in enumerate(keypad_z):
        for col, x in enumerate(keypad_x):
            _add_button_pad(
                body,
                x=x,
                z=z,
                radius=0.0046,
                thickness=0.0026,
                material=rubber_gray,
                name=f"key_{row}_{col}",
            )

    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_length + 0.050)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.119)),
    )

    antenna = model.part("antenna")
    antenna.visual(
        Cylinder(radius=0.0026, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, -0.0025)),
        material=metal,
        name="lower_stage",
    )
    antenna.visual(
        Cylinder(radius=0.00155, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, 0.0875)),
        material=metal,
        name="upper_stage",
    )
    antenna.visual(
        Sphere(radius=0.0019),
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        material=metal,
        name="tip",
    )
    antenna.inertial = Inertial.from_geometry(
        Box((0.008, 0.008, 0.160)),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
    )

    battery_door = model.part("battery_door")
    battery_door.visual(
        Box((0.043, 0.0022, 0.106)),
        origin=Origin(xyz=(0.0215, 0.0011, 0.0)),
        material=battery_door_gray,
        name="door_panel",
    )
    battery_door.visual(
        Box((0.006, 0.0016, 0.020)),
        origin=Origin(xyz=(0.040, -0.0002, 0.0)),
        material=fascia_black,
        name="finger_lip",
    )
    battery_door.inertial = Inertial.from_geometry(
        Box((0.044, 0.004, 0.108)),
        mass=0.018,
        origin=Origin(xyz=(0.0215, 0.0010, 0.0)),
    )

    model.articulation(
        "body_to_antenna",
        ArticulationType.PRISMATIC,
        parent=body,
        child=antenna,
        origin=Origin(xyz=(0.0, 0.0, 0.237)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.18,
            lower=0.0,
            upper=0.028,
        ),
    )
    model.articulation(
        "body_to_battery_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=battery_door,
        origin=Origin(xyz=(-0.0215, -0.0120, rear_opening_center_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.4,
            lower=0.0,
            upper=2.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    antenna = object_model.get_part("antenna")
    battery_door = object_model.get_part("battery_door")
    antenna_slide = object_model.get_articulation("body_to_antenna")
    door_hinge = object_model.get_articulation("body_to_battery_door")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_within(
        antenna,
        body,
        axes="xy",
        inner_elem="lower_stage",
        outer_elem="antenna_sleeve",
        margin=0.0018,
        name="antenna lower stage stays centered in the sleeve",
    )
    ctx.expect_overlap(
        antenna,
        body,
        axes="z",
        elem_a="lower_stage",
        elem_b="antenna_sleeve",
        min_overlap=0.022,
        name="antenna remains inserted in the sleeve at rest",
    )

    rest_antenna_pos = ctx.part_world_position(antenna)
    antenna_upper = (
        antenna_slide.motion_limits.upper
        if antenna_slide.motion_limits is not None and antenna_slide.motion_limits.upper is not None
        else 0.0
    )
    with ctx.pose({antenna_slide: antenna_upper}):
        ctx.expect_within(
            antenna,
            body,
            axes="xy",
            inner_elem="lower_stage",
            outer_elem="antenna_sleeve",
            margin=0.0018,
            name="antenna lower stage stays centered when extended",
        )
        ctx.expect_overlap(
            antenna,
            body,
            axes="z",
            elem_a="lower_stage",
            elem_b="antenna_sleeve",
            min_overlap=0.018,
            name="antenna keeps retained insertion when extended",
        )
        extended_antenna_pos = ctx.part_world_position(antenna)
        ctx.fail_if_parts_overlap_in_current_pose(name="extended antenna stays clear of the remote body")

    ctx.check(
        "antenna extends upward",
        rest_antenna_pos is not None
        and extended_antenna_pos is not None
        and extended_antenna_pos[2] > rest_antenna_pos[2] + 0.02,
        details=f"rest={rest_antenna_pos}, extended={extended_antenna_pos}",
    )

    ctx.expect_overlap(
        battery_door,
        body,
        axes="xz",
        min_overlap=0.040,
        elem_a="door_panel",
        name="battery door covers the rear compartment opening",
    )
    ctx.expect_within(
        battery_door,
        body,
        axes="xz",
        inner_elem="door_panel",
        margin=0.007,
        name="battery door stays within the rear body footprint",
    )

    closed_door_aabb = ctx.part_world_aabb(battery_door)
    door_upper = (
        door_hinge.motion_limits.upper
        if door_hinge.motion_limits is not None and door_hinge.motion_limits.upper is not None
        else 0.0
    )
    with ctx.pose({door_hinge: door_upper}):
        opened_door_aabb = ctx.part_world_aabb(battery_door)
        ctx.fail_if_parts_overlap_in_current_pose(name="opened battery door clears the body")

    ctx.check(
        "battery door swings outward from the rear face",
        closed_door_aabb is not None
        and opened_door_aabb is not None
        and opened_door_aabb[0][1] < closed_door_aabb[0][1] - 0.015,
        details=f"closed={closed_door_aabb}, opened={opened_door_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
