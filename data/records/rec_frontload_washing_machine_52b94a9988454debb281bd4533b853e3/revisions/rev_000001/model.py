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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _shell_mesh(
    name: str,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    segments: int = 72,
) -> object:
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=segments,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        name,
    )


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="builtin_undercounter_washing_machine")

    body_white = model.material("body_white", rgba=(0.93, 0.94, 0.95, 1.0))
    trim_white = model.material("trim_white", rgba=(0.97, 0.97, 0.98, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.28, 0.34, 0.40, 0.38))
    rubber_gray = model.material("rubber_gray", rgba=(0.16, 0.17, 0.18, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.60, 0.63, 0.67, 1.0))
    drum_metal = model.material("drum_metal", rgba=(0.72, 0.75, 0.78, 1.0))
    control_black = model.material("control_black", rgba=(0.09, 0.10, 0.11, 1.0))

    body_width = 0.595
    body_depth = 0.540
    body_height = 0.820
    side_thickness = 0.018
    top_thickness = 0.015
    bottom_thickness = 0.020
    back_thickness = 0.012
    front_thickness = 0.018
    front_x = body_depth * 0.5
    back_x = -body_depth * 0.5

    door_center_z = 0.430
    opening_box_width = 0.440
    opening_box_height = 0.440
    opening_half_w = opening_box_width * 0.5
    side_margin = (body_width - opening_box_width) * 0.5
    top_band_height = body_height - (door_center_z + opening_box_height * 0.5)
    bottom_band_height = door_center_z - (opening_box_height * 0.5)

    hinge_axis_y = -0.265
    door_ring_center_y = 0.265

    body = model.part("body")
    body.visual(
        Box((body_depth, side_thickness, body_height)),
        origin=Origin(xyz=(0.0, -0.5 * body_width + 0.5 * side_thickness, body_height * 0.5)),
        material=body_white,
        name="left_side_panel",
    )
    body.visual(
        Box((body_depth, side_thickness, body_height)),
        origin=Origin(xyz=(0.0, 0.5 * body_width - 0.5 * side_thickness, body_height * 0.5)),
        material=body_white,
        name="right_side_panel",
    )
    body.visual(
        Box((body_depth, body_width - 2.0 * side_thickness, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, body_height - 0.5 * top_thickness)),
        material=body_white,
        name="top_panel",
    )
    body.visual(
        Box((body_depth, body_width - 2.0 * side_thickness, bottom_thickness)),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * bottom_thickness)),
        material=body_white,
        name="bottom_panel",
    )
    body.visual(
        Box((back_thickness, body_width - 2.0 * side_thickness, body_height - 0.040)),
        origin=Origin(
            xyz=(back_x + 0.5 * back_thickness, 0.0, 0.5 * (body_height - 0.040) + 0.020)
        ),
        material=body_white,
        name="rear_panel",
    )
    body.visual(
        Box((front_thickness, side_margin, body_height)),
        origin=Origin(
            xyz=(
                front_x - 0.5 * front_thickness,
                -opening_half_w - 0.5 * side_margin,
                body_height * 0.5,
            )
        ),
        material=body_white,
        name="front_left_stile",
    )
    body.visual(
        Box((front_thickness, side_margin, body_height)),
        origin=Origin(
            xyz=(
                front_x - 0.5 * front_thickness,
                opening_half_w + 0.5 * side_margin,
                body_height * 0.5,
            )
        ),
        material=body_white,
        name="front_right_stile",
    )
    body.visual(
        Box((front_thickness, body_width, top_band_height)),
        origin=Origin(
            xyz=(front_x - 0.5 * front_thickness, 0.0, body_height - 0.5 * top_band_height)
        ),
        material=body_white,
        name="front_top_rail",
    )
    body.visual(
        Box((front_thickness, body_width, bottom_band_height)),
        origin=Origin(xyz=(front_x - 0.5 * front_thickness, 0.0, 0.5 * bottom_band_height)),
        material=body_white,
        name="front_bottom_rail",
    )
    body.visual(
        Box((0.070, body_width - 0.100, 0.070)),
        origin=Origin(xyz=(front_x - 0.095, 0.0, 0.035)),
        material=control_black,
        name="toe_kick_shadow",
    )
    body.visual(
        Box((0.012, 0.220, 0.055)),
        origin=Origin(xyz=(front_x + 0.006, 0.105, body_height - 0.065)),
        material=control_black,
        name="program_panel",
    )
    body.visual(
        _shell_mesh(
            "boot_gasket_mesh",
            outer_profile=[
                (0.176, -0.024),
                (0.196, -0.018),
                (0.212, -0.004),
                (0.218, 0.018),
                (0.208, 0.024),
            ],
            inner_profile=[
                (0.155, -0.014),
                (0.168, -0.008),
                (0.178, 0.004),
                (0.182, 0.016),
            ],
        ),
        origin=Origin(xyz=(0.236, 0.0, door_center_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_gray,
        name="boot_gasket",
    )
    body.visual(
        Box((0.020, 0.050, 0.110)),
        origin=Origin(xyz=(0.251, -0.220, door_center_z)),
        material=body_white,
        name="left_boot_bridge",
    )
    body.visual(
        Box((0.020, 0.050, 0.110)),
        origin=Origin(xyz=(0.251, 0.220, door_center_z)),
        material=body_white,
        name="right_boot_bridge",
    )
    body.visual(
        Box((0.020, 0.090, 0.026)),
        origin=Origin(xyz=(0.251, 0.0, door_center_z + 0.205)),
        material=body_white,
        name="top_boot_bridge",
    )
    body.visual(
        Box((0.020, 0.090, 0.026)),
        origin=Origin(xyz=(0.251, 0.0, door_center_z - 0.219)),
        material=body_white,
        name="bottom_boot_bridge",
    )
    body.visual(
        Cylinder(radius=0.055, length=0.040),
        origin=Origin(xyz=(-0.270, 0.0, door_center_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="rear_bearing_housing",
    )
    body.visual(
        Box((0.016, 0.036, 0.082)),
        origin=Origin(xyz=(front_x - 0.008, hinge_axis_y - 0.014, door_center_z + 0.074)),
        material=body_white,
        name="upper_hinge_mount",
    )
    body.visual(
        Box((0.016, 0.036, 0.082)),
        origin=Origin(xyz=(front_x - 0.008, hinge_axis_y - 0.014, door_center_z - 0.074)),
        material=body_white,
        name="lower_hinge_mount",
    )
    body.visual(
        Cylinder(radius=0.011, length=0.030),
        origin=Origin(xyz=(front_x, hinge_axis_y, door_center_z + 0.095)),
        material=hinge_metal,
        name="upper_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=0.011, length=0.030),
        origin=Origin(xyz=(front_x, hinge_axis_y, door_center_z - 0.095)),
        material=hinge_metal,
        name="lower_hinge_barrel",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_depth, body_width, body_height)),
        mass=52.0,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )

    drum = model.part("drum")
    drum.visual(
        _shell_mesh(
            "drum_shell_mesh",
            outer_profile=[
                (0.214, -0.210),
                (0.218, -0.196),
                (0.218, 0.175),
                (0.224, 0.199),
                (0.224, 0.210),
            ],
            inner_profile=[
                (0.196, -0.196),
                (0.198, -0.184),
                (0.198, 0.186),
                (0.204, 0.202),
            ],
        ),
        origin=Origin(xyz=(0.240, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=drum_metal,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.200, length=0.014),
        origin=Origin(xyz=(0.036, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=drum_metal,
        name="rear_hub_plate",
    )
    drum.visual(
        Cylinder(radius=0.080, length=0.028),
        origin=Origin(xyz=(0.024, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="rear_spider_hub",
    )
    drum.visual(
        _shell_mesh(
            "front_rim_mesh",
            outer_profile=[
                (0.190, -0.010),
                (0.212, -0.010),
                (0.220, 0.006),
                (0.220, 0.012),
            ],
            inner_profile=[
                (0.176, -0.006),
                (0.184, -0.002),
                (0.188, 0.010),
            ],
            segments=64,
        ),
        origin=Origin(xyz=(0.435, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=drum_metal,
        name="front_rim",
    )
    drum.visual(
        Cylinder(radius=0.030, length=0.045),
        origin=Origin(xyz=(0.0225, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="drive_shaft",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        drum.visual(
            Box((0.300, 0.034, 0.055)),
            origin=Origin(
                xyz=(0.240, 0.182 * math.sin(angle), 0.182 * math.cos(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=drum_metal,
            name=f"paddle_{index}",
        )
    drum.inertial = Inertial.from_geometry(
        Box((0.490, 0.450, 0.450)),
        mass=8.0,
        origin=Origin(xyz=(0.240, 0.0, 0.0)),
    )

    door = model.part("door")
    door.visual(
        _shell_mesh(
            "door_outer_ring_mesh",
            outer_profile=[
                (0.176, -0.024),
                (0.214, -0.024),
                (0.232, -0.012),
                (0.238, 0.008),
                (0.234, 0.020),
                (0.194, 0.024),
            ],
            inner_profile=[
                (0.162, -0.010),
                (0.170, -0.004),
                (0.176, 0.010),
                (0.176, 0.018),
            ],
        ),
        origin=Origin(xyz=(0.024, door_ring_center_y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_white,
        name="outer_ring",
    )
    door.visual(
        _shell_mesh(
            "door_inner_bezel_mesh",
            outer_profile=[
                (0.165, -0.010),
                (0.182, -0.008),
                (0.190, 0.002),
                (0.190, 0.014),
            ],
            inner_profile=[
                (0.150, -0.004),
                (0.162, 0.002),
                (0.168, 0.012),
            ],
            segments=64,
        ),
        origin=Origin(xyz=(0.012, door_ring_center_y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_white,
        name="inner_bezel",
    )
    door.visual(
        Cylinder(radius=0.166, length=0.010),
        origin=Origin(xyz=(0.017, door_ring_center_y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=smoked_glass,
        name="glass_lens",
    )
    door.visual(
        Box((0.018, 0.040, 0.072)),
        origin=Origin(xyz=(0.014, 0.040, 0.072)),
        material=trim_white,
        name="upper_hinge_leaf",
    )
    door.visual(
        Box((0.018, 0.040, 0.072)),
        origin=Origin(xyz=(0.014, 0.040, -0.072)),
        material=trim_white,
        name="lower_hinge_leaf",
    )
    door.visual(
        Cylinder(radius=0.011, length=0.036),
        origin=Origin(xyz=(0.012, 0.018, 0.062)),
        material=hinge_metal,
        name="upper_hinge_knuckle",
    )
    door.visual(
        Cylinder(radius=0.011, length=0.036),
        origin=Origin(xyz=(0.012, 0.018, -0.062)),
        material=hinge_metal,
        name="lower_hinge_knuckle",
    )
    door.visual(
        Cylinder(radius=0.007, length=0.042),
        origin=Origin(xyz=(0.034, 0.420, 0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="handle_standoff_upper",
    )
    door.visual(
        Cylinder(radius=0.007, length=0.042),
        origin=Origin(xyz=(0.034, 0.420, -0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="handle_standoff_lower",
    )
    door.visual(
        Cylinder(radius=0.010, length=0.160),
        origin=Origin(xyz=(0.055, 0.435, 0.0)),
        material=hinge_metal,
        name="handle_bar",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.080, 0.510, 0.500)),
        mass=4.5,
        origin=Origin(xyz=(0.030, door_ring_center_y, 0.0)),
    )

    model.articulation(
        "body_to_drum",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum,
        origin=Origin(xyz=(-0.250, 0.0, door_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=12.0),
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(front_x, hinge_axis_y, door_center_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=0.0,
            upper=1.85,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    drum_spin = object_model.get_articulation("body_to_drum")
    door_hinge = object_model.get_articulation("body_to_door")

    boot_gasket = body.get_visual("boot_gasket")
    rear_bearing = body.get_visual("rear_bearing_housing")
    upper_barrel = body.get_visual("upper_hinge_barrel")
    lower_barrel = body.get_visual("lower_hinge_barrel")
    door_ring = door.get_visual("outer_ring")
    door_glass = door.get_visual("glass_lens")
    upper_knuckle = door.get_visual("upper_hinge_knuckle")
    lower_knuckle = door.get_visual("lower_hinge_knuckle")
    drum_rim = drum.get_visual("front_rim")
    drive_shaft = drum.get_visual("drive_shaft")

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

    door_kind = str(getattr(door_hinge.joint_type, "value", door_hinge.joint_type)).lower()
    drum_kind = str(getattr(drum_spin.joint_type, "value", drum_spin.joint_type)).lower()

    ctx.check(
        "door uses a left-edge revolute hinge axis",
        door_kind.endswith("revolute")
        and tuple(door_hinge.axis) == (0.0, 0.0, -1.0)
        and door_hinge.motion_limits is not None
        and door_hinge.motion_limits.lower == 0.0
        and door_hinge.motion_limits.upper is not None
        and door_hinge.motion_limits.upper > 1.5,
        details=f"type={door_kind}, axis={door_hinge.axis}, limits={door_hinge.motion_limits}",
    )
    ctx.check(
        "drum uses a continuous front-back axle",
        drum_kind.endswith("continuous")
        and tuple(drum_spin.axis) == (1.0, 0.0, 0.0)
        and drum_spin.motion_limits is not None
        and drum_spin.motion_limits.lower is None
        and drum_spin.motion_limits.upper is None,
        details=f"type={drum_kind}, axis={drum_spin.axis}, limits={drum_spin.motion_limits}",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_overlap(
            door,
            body,
            axes="yz",
            elem_a=door_glass,
            elem_b=boot_gasket,
            min_overlap=0.320,
            name="door porthole aligns with the cabinet boot gasket",
        )
        ctx.expect_gap(
            door,
            body,
            axis="x",
            positive_elem=door_glass,
            negative_elem=boot_gasket,
            min_gap=0.015,
            max_gap=0.040,
            name="door glass sits just ahead of the boot gasket",
        )
        ctx.expect_contact(
            body,
            door,
            elem_a=upper_barrel,
            elem_b=upper_knuckle,
            contact_tol=1e-6,
            name="upper hinge knuckle bears on the cabinet barrel",
        )
        ctx.expect_contact(
            body,
            door,
            elem_a=lower_barrel,
            elem_b=lower_knuckle,
            contact_tol=1e-6,
            name="lower hinge knuckle bears on the cabinet barrel",
        )
        ctx.expect_overlap(
            drum,
            body,
            axes="yz",
            elem_a=drum_rim,
            elem_b=boot_gasket,
            min_overlap=0.340,
            name="drum opening stays centered behind the porthole",
        )
        ctx.expect_gap(
            body,
            drum,
            axis="x",
            positive_elem=boot_gasket,
            negative_elem=drum_rim,
            min_gap=0.010,
            max_gap=0.060,
            name="drum front rim sits just behind the door boot",
        )
        ctx.expect_gap(
            drum,
            body,
            axis="x",
            positive_elem=drive_shaft,
            negative_elem=rear_bearing,
            max_gap=0.001,
            max_penetration=0.0,
            name="drum axle meets the rear bearing housing",
        )

    closed_ring = ctx.part_element_world_aabb(door, elem=door_ring)
    upper_open = 0.0
    if door_hinge.motion_limits is not None and door_hinge.motion_limits.upper is not None:
        upper_open = door_hinge.motion_limits.upper
    with ctx.pose({door_hinge: upper_open}):
        open_ring = ctx.part_element_world_aabb(door, elem=door_ring)
        ctx.expect_gap(
            door,
            body,
            axis="x",
            positive_elem=door_ring,
            negative_elem=boot_gasket,
            min_gap=0.015,
            name="opened door clears forward of the cabinet front",
        )

    closed_center = _aabb_center(closed_ring)
    open_center = _aabb_center(open_ring)
    ctx.check(
        "door swings outward from the left edge",
        closed_center is not None
        and open_center is not None
        and open_center[0] > closed_center[0] + 0.120
        and open_center[1] < closed_center[1] - 0.050,
        details=f"closed_center={closed_center}, open_center={open_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
