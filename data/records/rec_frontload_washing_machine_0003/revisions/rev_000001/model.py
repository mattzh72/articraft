from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = Path(__file__).resolve().parent


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _circle_profile(radius: float, *, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width / 2.0
    half_h = height / 2.0
    return [
        (-half_w, -half_h),
        (half_w, -half_h),
        (half_w, half_h),
        (-half_w, half_h),
    ]


def _annulus_y_mesh(
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    *,
    segments: int = 72,
):
    return ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments=segments),
        [_circle_profile(inner_radius, segments=segments)],
        thickness,
        center=True,
    ).rotate_x(math.pi / 2.0)


def _panel_with_round_hole_y_mesh(
    width: float,
    height: float,
    thickness: float,
    *,
    hole_center_x: float,
    hole_center_z: float,
    hole_radius: float,
    hole_segments: int = 48,
):
    return ExtrudeWithHolesGeometry(
        _rect_profile(width, height),
        [[(x + hole_center_x, y + hole_center_z) for x, y in _circle_profile(hole_radius, segments=hole_segments)]],
        thickness,
        center=True,
    ).rotate_x(math.pi / 2.0)


def _open_cylinder_y_mesh(radius: float, length: float, *, radial_segments: int = 72):
    return CylinderGeometry(
        radius=radius,
        height=length,
        radial_segments=radial_segments,
        closed=False,
    ).rotate_x(math.pi / 2.0)


def _door_handle_mesh():
    return wire_from_points(
        [
            (-0.356, 0.018, 0.042),
            (-0.380, 0.038, 0.042),
            (-0.396, 0.044, 0.018),
            (-0.396, 0.044, -0.018),
            (-0.380, 0.038, -0.042),
            (-0.356, 0.018, -0.042),
        ],
        radius=0.0045,
        radial_segments=16,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.012,
        corner_segments=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_front_loader", assets=ASSETS)

    cabinet_white = model.material("cabinet_white", rgba=(0.96, 0.97, 0.98, 1.0))
    panel_gray = model.material("panel_gray", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.16, 0.17, 0.19, 1.0))
    chrome = model.material("chrome", rgba=(0.74, 0.77, 0.81, 1.0))
    drum_steel = model.material("drum_steel", rgba=(0.67, 0.70, 0.74, 1.0))
    glass = model.material("glass", rgba=(0.62, 0.76, 0.86, 0.30))
    display_black = model.material("display_black", rgba=(0.06, 0.07, 0.08, 1.0))

    body_width = 0.520
    body_depth = 0.440
    body_height = 0.760
    side_thickness = 0.012
    back_thickness = 0.010
    front_thickness = 0.012
    opening_center_z = 0.348
    opening_radius = 0.155
    top_band_height = body_height - (opening_center_z + opening_radius)
    lower_band_height = opening_center_z - opening_radius
    top_panel_center_z = (body_height + opening_center_z + opening_radius) / 2.0
    inner_width = body_width - (2.0 * side_thickness)
    jamb_width = (inner_width - (2.0 * opening_radius)) / 2.0
    front_panel_y = (body_depth / 2.0) - (front_thickness / 2.0)
    hinge_axis_x = opening_radius + 0.051
    drum_center_y = 0.010
    door_joint_y = (body_depth / 2.0) + 0.011
    dial_x = 0.150
    dial_z = 0.689

    body = model.part("body")
    body.visual(
        Box((side_thickness, body_depth, body_height)),
        origin=Origin(xyz=(-(body_width / 2.0) + (side_thickness / 2.0), 0.0, body_height / 2.0)),
        material=cabinet_white,
        name="left_side",
    )
    body.visual(
        Box((side_thickness, body_depth, body_height)),
        origin=Origin(xyz=((body_width / 2.0) - (side_thickness / 2.0), 0.0, body_height / 2.0)),
        material=cabinet_white,
        name="right_side",
    )
    body.visual(
        Box((inner_width, body_depth, side_thickness)),
        origin=Origin(xyz=(0.0, 0.0, body_height - (side_thickness / 2.0))),
        material=cabinet_white,
        name="top_panel",
    )
    body.visual(
        Box((inner_width, body_depth - 0.020, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=cabinet_white,
        name="bottom_plinth",
    )
    body.visual(
        Box((inner_width, back_thickness, body_height - side_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -(body_depth / 2.0) + (back_thickness / 2.0),
                (body_height - side_thickness) / 2.0,
            )
        ),
        material=cabinet_white,
        name="back_panel",
    )
    body.visual(
        _save_mesh(
            "front_top_panel.obj",
            _panel_with_round_hole_y_mesh(
                body_width,
                top_band_height,
                front_thickness,
                hole_center_x=dial_x,
                hole_center_z=dial_z - top_panel_center_z,
                hole_radius=0.024,
            ),
        ),
        origin=Origin(xyz=(0.0, front_panel_y, top_panel_center_z)),
        material=cabinet_white,
        name="front_top_panel",
    )
    body.visual(
        Box((body_width, front_thickness, lower_band_height)),
        origin=Origin(
            xyz=(
                0.0,
                front_panel_y,
                lower_band_height / 2.0,
            )
        ),
        material=cabinet_white,
        name="front_bottom_panel",
    )
    body.visual(
        Box((jamb_width, front_thickness, 2.0 * opening_radius)),
        origin=Origin(
            xyz=(
                -(opening_radius + (jamb_width / 2.0)),
                front_panel_y,
                opening_center_z,
            )
        ),
        material=cabinet_white,
        name="front_left_jamb",
    )
    body.visual(
        Box((jamb_width, front_thickness, 2.0 * opening_radius)),
        origin=Origin(
            xyz=(
                opening_radius + (jamb_width / 2.0),
                front_panel_y,
                opening_center_z,
            )
        ),
        material=cabinet_white,
        name="front_right_jamb",
    )
    body.visual(
        _save_mesh("opening_gasket.obj", _annulus_y_mesh(0.166, 0.145, 0.020)),
        origin=Origin(xyz=(0.0, 0.208, opening_center_z)),
        material=dark_gray,
        name="opening_gasket",
    )
    body.visual(
        Box((0.292, 0.008, 0.102)),
        origin=Origin(xyz=(-0.080, 0.222, 0.695)),
        material=panel_gray,
        name="control_panel",
    )
    body.visual(
        _save_mesh(
            "dial_fascia.obj",
            _panel_with_round_hole_y_mesh(
                0.126,
                0.102,
                0.008,
                hole_center_x=0.0,
                hole_center_z=0.0,
                hole_radius=0.018,
            ),
        ),
        origin=Origin(xyz=(dial_x, 0.218, dial_z)),
        material=panel_gray,
        name="dial_fascia",
    )
    body.visual(
        Box((0.118, 0.004, 0.042)),
        origin=Origin(xyz=(-0.082, 0.220, 0.700)),
        material=display_black,
        name="display_window",
    )
    body.visual(
        Box((0.060, 0.004, 0.012)),
        origin=Origin(xyz=(-0.082, 0.220, 0.667)),
        material=chrome,
        name="display_trim",
    )
    body.visual(
        Cylinder(radius=0.026, length=0.084),
        origin=Origin(xyz=(0.0, -0.173, opening_center_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="rear_bearing",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(dial_x, 0.213, dial_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=panel_gray,
        name="dial_collar",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=27.0,
        origin=Origin(xyz=(0.0, 0.0, body_height / 2.0)),
    )

    drum = model.part("drum")
    drum.visual(
        _save_mesh("drum_shell.obj", _open_cylinder_y_mesh(0.150, 0.224)),
        material=drum_steel,
        name="drum_shell",
    )
    drum.visual(
        _save_mesh("drum_front_rim.obj", _annulus_y_mesh(0.153, 0.120, 0.016)),
        origin=Origin(xyz=(0.0, 0.104, 0.0)),
        material=chrome,
        name="drum_front_rim",
    )
    drum.visual(
        _save_mesh("drum_rear_plate.obj", _annulus_y_mesh(0.148, 0.034, 0.016)),
        origin=Origin(xyz=(0.0, -0.104, 0.0)),
        material=drum_steel,
        name="drum_rear_plate",
    )
    drum.visual(
        Cylinder(radius=0.036, length=0.024),
        origin=Origin(xyz=(0.0, -0.100, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="drum_hub",
    )
    for index, angle in enumerate((0.0, (2.0 * math.pi) / 3.0, (4.0 * math.pi) / 3.0), start=1):
        drum.visual(
            Box((0.114, 0.208, 0.012)),
            origin=Origin(
                xyz=(0.093 * math.cos(angle), 0.0, 0.093 * math.sin(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=drum_steel,
            name=f"drum_paddle_{index}",
        )
    drum.visual(
        Cylinder(radius=0.024, length=0.058),
        origin=Origin(xyz=(0.0, -0.141, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="drum_axle",
    )
    drum.inertial = Inertial.from_geometry(
        Cylinder(radius=0.150, length=0.224),
        mass=6.8,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    door = model.part("door")
    door.visual(
        _save_mesh("door_outer_ring.obj", _annulus_y_mesh(0.186, 0.134, 0.028)),
        origin=Origin(xyz=(-hinge_axis_x, 0.002, 0.0)),
        material=cabinet_white,
        name="door_outer_ring",
    )
    door.visual(
        _save_mesh("door_inner_trim.obj", _annulus_y_mesh(0.166, 0.134, 0.010)),
        origin=Origin(xyz=(-hinge_axis_x, -0.008, 0.0)),
        material=chrome,
        name="door_inner_trim",
    )
    door.visual(
        _save_mesh("door_glass_retainer.obj", _annulus_y_mesh(0.134, 0.128, 0.008)),
        origin=Origin(xyz=(-hinge_axis_x, -0.017, 0.0)),
        material=panel_gray,
        name="door_glass_retainer",
    )
    door.visual(
        Cylinder(radius=0.128, length=0.012),
        origin=Origin(xyz=(-hinge_axis_x, -0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="door_glass",
    )
    door.visual(
        Box((0.024, 0.014, 0.210)),
        origin=Origin(xyz=(-0.010, 0.000, 0.0)),
        material=cabinet_white,
        name="door_hinge_backplate",
    )
    door.visual(
        Cylinder(radius=0.008, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=panel_gray,
        name="door_hinge_upper",
    )
    door.visual(
        Cylinder(radius=0.008, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
        material=panel_gray,
        name="door_hinge_lower",
    )
    door.visual(
        Box((0.056, 0.022, 0.108)),
        origin=Origin(xyz=(-0.366, 0.008, 0.0)),
        material=chrome,
        name="latch_base",
    )
    door.visual(
        Cylinder(radius=0.0055, length=0.024),
        origin=Origin(xyz=(-0.350, 0.016, 0.034), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="latch_post_upper",
    )
    door.visual(
        Cylinder(radius=0.0055, length=0.024),
        origin=Origin(xyz=(-0.350, 0.016, -0.034), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="latch_post_lower",
    )
    door.visual(
        _save_mesh("door_handle.obj", _door_handle_mesh()),
        material=chrome,
        name="latch_handle",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.380, 0.090, 0.380)),
        mass=2.6,
        origin=Origin(xyz=(-hinge_axis_x, 0.006, 0.0)),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.036, length=0.018),
        origin=Origin(xyz=(0.0, 0.021, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=panel_gray,
        name="dial_knob",
    )
    dial.visual(
        Cylinder(radius=0.038, length=0.006),
        origin=Origin(xyz=(0.0, 0.011, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="dial_bezel",
    )
    dial.visual(
        Cylinder(radius=0.014, length=0.020),
        origin=Origin(xyz=(0.0, 0.000, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="dial_shaft",
    )
    dial.visual(
        Box((0.006, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, 0.032, 0.021)),
        material=cabinet_white,
        name="dial_pointer",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.036, length=0.020),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_drum",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum,
        origin=Origin(xyz=(0.0, drum_center_y, opening_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=16.0),
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_axis_x, door_joint_y, opening_center_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.2, lower=0.0, upper=1.95),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(dial_x, 0.213, dial_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)

    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")

    drum_spin = object_model.get_articulation("body_to_drum")
    door_hinge = object_model.get_articulation("body_to_door")
    dial_spin = object_model.get_articulation("body_to_dial")

    front_top_panel = body.get_visual("front_top_panel")
    opening_gasket = body.get_visual("opening_gasket")
    rear_bearing = body.get_visual("rear_bearing")
    dial_collar = body.get_visual("dial_collar")
    control_panel = body.get_visual("control_panel")

    drum_shell = drum.get_visual("drum_shell")
    drum_front_rim = drum.get_visual("drum_front_rim")
    drum_axle = drum.get_visual("drum_axle")

    door_outer_ring = door.get_visual("door_outer_ring")
    door_glass = door.get_visual("door_glass")
    door_hinge_upper = door.get_visual("door_hinge_upper")
    door_hinge_lower = door.get_visual("door_hinge_lower")
    latch_handle = door.get_visual("latch_handle")

    dial_knob = dial.get_visual("dial_knob")
    dial_shaft = dial.get_visual("dial_shaft")

    ctx.allow_overlap(
        body,
        drum,
        elem_a=rear_bearing,
        elem_b=drum_axle,
        reason="rear bearing encloses the rotating drum axle",
    )
    ctx.allow_overlap(
        body,
        dial,
        elem_a=dial_collar,
        elem_b=dial_shaft,
        reason="dial shaft rotates inside the fixed front-panel collar",
    )
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.check(
        "door_joint_axis",
        tuple(door_hinge.axis) == (0.0, 0.0, -1.0),
        f"expected right-hand door hinge axis (0, 0, -1), got {door_hinge.axis}",
    )
    ctx.check(
        "drum_joint_axis",
        tuple(drum_spin.axis) == (0.0, 1.0, 0.0),
        f"expected drum axle axis (0, 1, 0), got {drum_spin.axis}",
    )
    ctx.check(
        "dial_joint_axis",
        tuple(dial_spin.axis) == (0.0, 1.0, 0.0),
        f"expected selector dial axis (0, 1, 0), got {dial_spin.axis}",
    )

    limits = door_hinge.motion_limits
    ctx.check(
        "door_open_range",
        limits is not None and limits.lower == 0.0 and limits.upper is not None and 1.7 <= limits.upper <= 2.0,
        "door hinge should open from closed to roughly 110 degrees",
    )

    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        elem_a=door_outer_ring,
        elem_b=opening_gasket,
        min_overlap=0.30,
    )
    ctx.expect_gap(
        door,
        body,
        axis="y",
        min_gap=0.001,
        max_gap=0.010,
        positive_elem=door_outer_ring,
        negative_elem=opening_gasket,
    )
    ctx.expect_overlap(
        door,
        drum,
        axes="xz",
        elem_a=door_glass,
        elem_b=drum_shell,
        min_overlap=0.24,
    )
    ctx.expect_gap(
        door,
        drum,
        axis="y",
        min_gap=0.050,
        max_gap=0.090,
        positive_elem=door_glass,
        negative_elem=drum_shell,
    )
    ctx.expect_overlap(
        body,
        drum,
        axes="xz",
        elem_a=opening_gasket,
        elem_b=drum_front_rim,
        min_overlap=0.28,
    )
    ctx.expect_overlap(
        dial,
        body,
        axes="xz",
        elem_a=dial_knob,
        elem_b=front_top_panel,
        min_overlap=0.060,
    )
    ctx.expect_gap(
        dial,
        body,
        axis="y",
        min_gap=0.0,
        max_gap=0.028,
        positive_elem=dial_knob,
        negative_elem=front_top_panel,
        name="dial_protrudes_from_panel",
    )

    if limits is not None and limits.upper is not None:
        with ctx.pose({door_hinge: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_open_no_unintended_overlap")
            ctx.fail_if_isolated_parts(name="door_open_supported")
            ctx.expect_gap(
                door,
                body,
                axis="y",
                min_gap=0.140,
                positive_elem=latch_handle,
                negative_elem=front_top_panel,
                name="door_handle_swings_forward",
            )

    with ctx.pose({drum_spin: math.pi / 2.0, dial_spin: 1.4}):
        ctx.fail_if_parts_overlap_in_current_pose(name="rotating_controls_no_overlap")
        ctx.fail_if_isolated_parts(name="rotating_controls_supported")
        ctx.expect_overlap(
            body,
            drum,
            axes="xz",
            elem_a=opening_gasket,
            elem_b=drum_front_rim,
            min_overlap=0.28,
            name="drum_stays_centered_on_axle",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
