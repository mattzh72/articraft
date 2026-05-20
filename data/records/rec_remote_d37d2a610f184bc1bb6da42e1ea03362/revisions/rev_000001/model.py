from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_side_loft,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flip_cover_tv_remote")

    body_plastic = model.material("body_plastic", rgba=(0.18, 0.19, 0.20, 1.0))
    bezel_black = model.material("bezel_black", rgba=(0.09, 0.10, 0.11, 1.0))
    button_rubber = model.material("button_rubber", rgba=(0.20, 0.21, 0.23, 1.0))
    smoked_cover = model.material("smoked_cover", rgba=(0.20, 0.28, 0.31, 0.40))
    hinge_plastic = model.material("hinge_plastic", rgba=(0.13, 0.14, 0.15, 1.0))
    accent_rib = model.material("accent_rib", rgba=(0.34, 0.35, 0.37, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((0.053, 0.205, 0.022)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
    )

    body_shell = superellipse_side_loft(
        [
            (0.102, -0.0082, 0.0106, 0.0460),
            (0.072, -0.0088, 0.0110, 0.0495),
            (0.024, -0.0093, 0.0113, 0.0520),
            (-0.034, -0.0091, 0.0110, 0.0525),
            (-0.080, -0.0085, 0.0100, 0.0500),
            (-0.102, -0.0074, 0.0087, 0.0465),
        ],
        exponents=3.1,
        segments=56,
    )
    body.visual(_mesh("remote_body_shell", body_shell), material=body_plastic, name="body_shell")
    body.visual(
        Box((0.040, 0.164, 0.0016)),
        origin=Origin(xyz=(0.0, -0.006, 0.0097)),
        material=bezel_black,
        name="button_face",
    )
    body.visual(
        Box((0.032, 0.040, 0.0012)),
        origin=Origin(xyz=(0.0, 0.046, 0.0111)),
        material=button_rubber,
        name="upper_function_bank",
    )
    body.visual(
        Cylinder(radius=0.011, length=0.0016),
        origin=Origin(xyz=(0.0, 0.002, 0.0113)),
        material=button_rubber,
        name="nav_pad",
    )
    body.visual(
        Box((0.034, 0.058, 0.0012)),
        origin=Origin(xyz=(0.0, -0.058, 0.0111)),
        material=button_rubber,
        name="numeric_bank",
    )
    body.visual(
        Cylinder(radius=0.0055, length=0.0014),
        origin=Origin(xyz=(0.0, 0.080, 0.01115)),
        material=button_rubber,
        name="power_button",
    )
    for side_x in (-0.012, 0.012):
        body.visual(
            Box((0.010, 0.022, 0.0014)),
            origin=Origin(xyz=(side_x, 0.048, 0.0112)),
            material=button_rubber,
            name=f"soft_key_{'left' if side_x < 0.0 else 'right'}",
        )
    body.visual(
        Cylinder(radius=0.0045, length=0.0014),
        origin=Origin(xyz=(0.0, 0.028, 0.0112)),
        material=accent_rib,
        name="ok_button",
    )
    for x_pos, y_pos in ((-0.012, 0.004), (0.012, 0.004), (0.0, 0.016), (0.0, -0.010)):
        body.visual(
            Box((0.010, 0.010, 0.0012)),
            origin=Origin(xyz=(x_pos, y_pos, 0.01115)),
            material=button_rubber,
            name=f"nav_key_{str(x_pos).replace('-', 'm').replace('.', '_')}_{str(y_pos).replace('-', 'm').replace('.', '_')}",
        )
    for x_pos in (-0.013, 0.013):
        body.visual(
            Box((0.008, 0.028, 0.0014)),
            origin=Origin(xyz=(x_pos, -0.024, 0.0112)),
            material=button_rubber,
            name=f"{'volume' if x_pos < 0.0 else 'channel'}_rocker",
        )
    for row_index, y_pos in enumerate((-0.048, -0.064, -0.080)):
        for col_index, x_pos in enumerate((-0.012, 0.0, 0.012)):
            body.visual(
                Cylinder(radius=0.0041, length=0.0014),
                origin=Origin(xyz=(x_pos, y_pos, 0.01115)),
                material=button_rubber,
                name=f"digit_{row_index}_{col_index}",
            )
    body.visual(
        Box((0.012, 0.008, 0.0014)),
        origin=Origin(xyz=(0.0, -0.084, 0.0112)),
        material=button_rubber,
        name="bottom_button",
    )
    body.visual(
        Box((0.040, 0.120, 0.0012)),
        origin=Origin(xyz=(0.0, -0.010, -0.0089)),
        material=bezel_black,
        name="battery_track_plane",
    )
    body.visual(
        Cylinder(radius=0.0031, length=0.010),
        origin=Origin(xyz=(-0.014, 0.100, 0.0105), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_plastic,
        name="left_hinge_knuckle",
    )
    body.visual(
        Cylinder(radius=0.0031, length=0.010),
        origin=Origin(xyz=(0.014, 0.100, 0.0105), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_plastic,
        name="right_hinge_knuckle",
    )

    flip_cover = model.part("flip_cover")
    flip_cover.inertial = Inertial.from_geometry(
        Box((0.046, 0.076, 0.007)),
        mass=0.025,
        origin=Origin(xyz=(0.0, -0.036, 0.003)),
    )
    cover_panel = ExtrudeGeometry(
        rounded_rect_profile(0.044, 0.072, 0.005, corner_segments=8),
        0.0026,
        center=True,
    )
    flip_cover.visual(
        _mesh("remote_flip_cover_panel", cover_panel),
        origin=Origin(xyz=(0.0, -0.037, 0.0035)),
        material=smoked_cover,
        name="cover_panel",
    )
    flip_cover.visual(
        Box((0.020, 0.004, 0.0012)),
        origin=Origin(xyz=(0.0, -0.073, 0.0040)),
        material=hinge_plastic,
        name="cover_lip",
    )
    flip_cover.visual(
        Cylinder(radius=0.0031, length=0.018),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_plastic,
        name="cover_hinge_barrel",
    )

    battery_door = model.part("battery_door")
    battery_door.inertial = Inertial.from_geometry(
        Box((0.040, 0.112, 0.003)),
        mass=0.020,
        origin=Origin(),
    )
    door_panel = ExtrudeGeometry(
        rounded_rect_profile(0.038, 0.110, 0.004, corner_segments=8),
        0.0018,
        center=True,
    )
    battery_door.visual(
        _mesh("remote_battery_door_panel", door_panel),
        material=body_plastic,
        name="battery_door_panel",
    )
    battery_door.visual(
        Box((0.018, 0.006, 0.0008)),
        origin=Origin(xyz=(0.0, -0.040, -0.0013)),
        material=accent_rib,
        name="thumb_rib",
    )
    battery_door.visual(
        Box((0.014, 0.005, 0.0008)),
        origin=Origin(xyz=(0.0, -0.052, -0.0013)),
        material=accent_rib,
        name="thumb_rib_lower",
    )

    model.articulation(
        "body_to_flip_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flip_cover,
        origin=Origin(xyz=(0.0, 0.100, 0.0105)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=0.0, upper=2.05),
    )
    model.articulation(
        "body_to_battery_door",
        ArticulationType.PRISMATIC,
        parent=body,
        child=battery_door,
        origin=Origin(xyz=(0.0, -0.010, -0.0104)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.10, lower=0.0, upper=0.028),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    flip_cover = object_model.get_part("flip_cover")
    battery_door = object_model.get_part("battery_door")
    cover_joint = object_model.get_articulation("body_to_flip_cover")
    door_joint = object_model.get_articulation("body_to_battery_door")

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

    ctx.expect_contact(
        flip_cover,
        body,
        name="flip cover remains mounted to the body at the hinge",
    )
    ctx.expect_overlap(
        flip_cover,
        body,
        axes="xy",
        elem_a="cover_panel",
        elem_b="upper_function_bank",
        min_overlap=0.025,
        name="closed cover spans the upper function buttons",
    )
    ctx.expect_gap(
        body,
        battery_door,
        axis="z",
        positive_elem="battery_track_plane",
        negative_elem="battery_door_panel",
        max_gap=0.0005,
        max_penetration=0.0,
        name="battery door closes flush against the rear track plane",
    )
    ctx.expect_within(
        battery_door,
        body,
        axes="x",
        inner_elem="battery_door_panel",
        outer_elem="battery_track_plane",
        margin=0.0015,
        name="battery door stays centered on the rear slide track",
    )

    with ctx.pose({door_joint: door_joint.motion_limits.upper}):
        ctx.expect_gap(
            body,
            battery_door,
            axis="z",
            positive_elem="battery_track_plane",
            negative_elem="battery_door_panel",
            max_gap=0.0005,
            max_penetration=0.0,
            name="battery door stays on the rear track while open",
        )
        ctx.expect_overlap(
            battery_door,
            body,
            axes="y",
            elem_a="battery_door_panel",
            elem_b="battery_track_plane",
            min_overlap=0.075,
            name="battery door retains overlap with the rear track at full extension",
        )

    closed_cover_aabb = ctx.part_element_world_aabb(flip_cover, elem="cover_panel")
    open_cover_aabb = None
    with ctx.pose({cover_joint: cover_joint.motion_limits.upper}):
        open_cover_aabb = ctx.part_element_world_aabb(flip_cover, elem="cover_panel")

    def _max_z(aabb):
        return None if aabb is None else aabb[1][2]

    ctx.check(
        "flip cover opens upward",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and _max_z(open_cover_aabb) is not None
        and _max_z(closed_cover_aabb) is not None
        and _max_z(open_cover_aabb) > _max_z(closed_cover_aabb) + 0.035,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    closed_door_pos = ctx.part_world_position(battery_door)
    opened_door_pos = None
    with ctx.pose({door_joint: door_joint.motion_limits.upper}):
        opened_door_pos = ctx.part_world_position(battery_door)
    ctx.check(
        "battery door slides toward the bottom of the remote",
        closed_door_pos is not None
        and opened_door_pos is not None
        and opened_door_pos[1] < closed_door_pos[1] - 0.020,
        details=f"closed={closed_door_pos}, open={opened_door_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
