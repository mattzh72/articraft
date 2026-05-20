from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
)


BASE_HEIGHT = 0.084
CUP_MOUNT_Z = 0.095
CUP_HEIGHT = 0.155
CUP_TOP_Z = CUP_MOUNT_Z + CUP_HEIGHT


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 48,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * i) / segments),
            cy + radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def _build_motor_base_mesh() -> MeshGeometry:
    body = LatheGeometry(
        [
            (0.0, 0.0),
            (0.059, 0.0),
            (0.070, 0.004),
            (0.078, 0.014),
            (0.079, 0.055),
            (0.074, 0.074),
            (0.064, 0.081),
            (0.052, BASE_HEIGHT),
            (0.0, BASE_HEIGHT),
        ],
        segments=72,
    )
    top_mount = CylinderGeometry(radius=0.048, height=0.008, radial_segments=48).translate(
        0.0,
        0.0,
        BASE_HEIGHT - 0.004,
    )
    foot_ring = CylinderGeometry(radius=0.064, height=0.006, radial_segments=48).translate(
        0.0,
        0.0,
        0.003,
    )
    return _merge_geometries(body, top_mount, foot_ring)


def _build_cup_mesh() -> MeshGeometry:
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.046, 0.000),
            (0.049, 0.022),
            (0.052, 0.072),
            (0.051, 0.116),
            (0.047, 0.148),
            (0.044, CUP_HEIGHT),
        ],
        [
            (0.041, 0.000),
            (0.044, 0.022),
            (0.047, 0.072),
            (0.046, 0.116),
            (0.042, 0.148),
            (0.039, CUP_HEIGHT),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    lower_thread_a = TorusGeometry(radius=0.0456, tube=0.0013, tubular_segments=56).translate(
        0.0,
        0.0,
        0.010,
    )
    lower_thread_b = TorusGeometry(radius=0.0466, tube=0.0012, tubular_segments=56).translate(
        0.0,
        0.0,
        0.018,
    )
    grip_band_a = TorusGeometry(radius=0.0496, tube=0.0016, tubular_segments=56).translate(
        0.0,
        0.0,
        0.050,
    )
    grip_band_b = TorusGeometry(radius=0.0500, tube=0.0016, tubular_segments=56).translate(
        0.0,
        0.0,
        0.088,
    )
    top_rim = TorusGeometry(radius=0.0415, tube=0.0015, tubular_segments=56).translate(
        0.0,
        0.0,
        CUP_HEIGHT - 0.001,
    )
    return _merge_geometries(
        shell,
        lower_thread_a,
        lower_thread_b,
        grip_band_a,
        grip_band_b,
        top_rim,
    )


def _build_blade_body_mesh() -> MeshGeometry:
    base_cap = LatheGeometry(
        [
            (0.0, -0.018),
            (0.055, -0.018),
            (0.055, -0.010),
            (0.047, -0.010),
            (0.044, -0.005),
            (0.040, -0.0015),
            (0.040, 0.0),
            (0.0, 0.0),
        ],
        segments=72,
    )
    center_hub = CylinderGeometry(radius=0.008, height=0.014, radial_segments=32).translate(
        0.0,
        0.0,
        0.007,
    )
    support_plate = CylinderGeometry(radius=0.017, height=0.0025, radial_segments=32).translate(
        0.0,
        0.0,
        0.00125,
    )
    mouth_gasket = LatheGeometry.from_shell_profiles(
        [
            (0.046, -0.0012),
            (0.046, 0.0),
        ],
        [
            (0.0395, -0.0012),
            (0.0395, 0.0),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    return _merge_geometries(base_cap, center_hub, support_plate, mouth_gasket)


def _build_blade_fin_mesh() -> MeshGeometry:
    section_a = [
        (0.005, -0.0025, 0.003),
        (0.012, -0.0015, 0.005),
        (0.012, 0.0015, 0.005),
        (0.005, 0.0025, 0.003),
    ]
    section_b = [
        (0.011, -0.0038, 0.0055),
        (0.020, -0.0012, 0.0088),
        (0.020, 0.0012, 0.0088),
        (0.011, 0.0038, 0.0055),
    ]
    section_c = [
        (0.018, -0.0020, 0.0065),
        (0.028, -0.0007, 0.0125),
        (0.028, 0.0007, 0.0125),
        (0.018, 0.0020, 0.0065),
    ]
    one_blade = repair_loft(section_loft([section_a, section_b, section_c]))
    blades = MeshGeometry()
    for angle in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0):
        blades.merge(one_blade.copy().rotate_z(angle))
    return blades


def _build_lid_body_mesh() -> MeshGeometry:
    deck = ExtrudeWithHolesGeometry(
        _circle_profile(0.043, segments=64),
        [_circle_profile(0.0065, center=(0.018, 0.0), segments=32)],
        0.004,
        center=False,
    )
    collar = LatheGeometry.from_shell_profiles(
        [
            (0.044, 0.000),
            (0.044, 0.010),
            (0.041, 0.014),
        ],
        [
            (0.036, 0.000),
            (0.036, 0.010),
            (0.034, 0.012),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    hinge_bridge = BoxGeometry((0.016, 0.022, 0.012)).translate(-0.026, 0.0, 0.020)
    knuckle_left = (
        CylinderGeometry(radius=0.0038, height=0.006, radial_segments=20)
        .rotate_x(math.pi / 2.0)
        .translate(-0.032, -0.007, 0.026)
    )
    knuckle_right = (
        CylinderGeometry(radius=0.0038, height=0.006, radial_segments=20)
        .rotate_x(math.pi / 2.0)
        .translate(-0.032, 0.007, 0.026)
    )
    return _merge_geometries(deck, collar, hinge_bridge, knuckle_left, knuckle_right)


def _build_spout_ring_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.0080, 0.000),
            (0.0100, 0.0015),
            (0.0100, 0.0035),
            (0.0080, 0.0045),
        ],
        [
            (0.0068, 0.000),
            (0.0068, 0.0045),
        ],
        segments=40,
        start_cap="flat",
        end_cap="flat",
    ).translate(0.018, 0.0, 0.004)


def _build_flip_cap_mesh() -> MeshGeometry:
    cover = ExtrudeGeometry(
        rounded_rect_profile(0.022, 0.018, 0.0045),
        0.0025,
        center=True,
    ).translate(0.050, 0.0, -0.014)
    thumb_tab = BoxGeometry((0.008, 0.010, 0.002)).translate(0.061, 0.0, -0.014)
    left_arm = BoxGeometry((0.050, 0.003, 0.003)).translate(0.025, -0.0075, -0.006)
    right_arm = BoxGeometry((0.050, 0.003, 0.003)).translate(0.025, 0.0075, -0.006)
    seal_stem = CylinderGeometry(radius=0.0024, height=0.010, radial_segments=18).translate(
        0.050,
        0.0,
        -0.019,
    )
    center_knuckle = (
        CylinderGeometry(radius=0.0036, height=0.014, radial_segments=20)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.0, 0.0)
    )
    return _merge_geometries(
        cover,
        thumb_tab,
        left_arm,
        right_arm,
        seal_stem,
        center_knuckle,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nutribullet_style_blender")

    base_charcoal = model.material("base_charcoal", rgba=(0.16, 0.16, 0.17, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.10, 0.10, 0.10, 1.0))
    smoke_cup = model.material("smoke_cup", rgba=(0.72, 0.76, 0.80, 0.36))
    blade_metal = model.material("blade_metal", rgba=(0.86, 0.88, 0.90, 1.0))
    lid_black = model.material("lid_black", rgba=(0.12, 0.12, 0.13, 1.0))
    gasket_grey = model.material("gasket_grey", rgba=(0.38, 0.40, 0.42, 1.0))

    motor_base = model.part("motor_base")
    motor_base.visual(
        mesh_from_geometry(_build_motor_base_mesh(), "motor_base_shell"),
        material=base_charcoal,
        name="base_shell",
    )
    motor_base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.080, length=BASE_HEIGHT),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
    )

    drive_coupler = model.part("drive_coupler")
    drive_coupler.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_plastic,
        name="drive_core",
    )
    drive_coupler.visual(
        Box((0.022, 0.004, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=dark_plastic,
        name="drive_lug_x",
    )
    drive_coupler.visual(
        Box((0.004, 0.022, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=dark_plastic,
        name="drive_lug_y",
    )
    drive_coupler.inertial = Inertial.from_geometry(
        Cylinder(radius=0.012, length=0.012),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )

    cup = model.part("cup")
    cup.visual(
        mesh_from_geometry(_build_cup_mesh(), "cup_shell"),
        material=smoke_cup,
        name="cup_shell",
    )
    cup.inertial = Inertial.from_geometry(
        Cylinder(radius=0.053, length=CUP_HEIGHT),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, CUP_HEIGHT / 2.0)),
    )

    blade_assembly = model.part("blade_assembly")
    blade_assembly.visual(
        mesh_from_geometry(
            _merge_geometries(_build_blade_body_mesh(), _build_blade_fin_mesh()),
            "blade_unit",
        ),
        material=gasket_grey,
        name="blade_unit",
    )
    blade_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=0.048, length=0.030),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
    )

    lid_collar = model.part("lid_collar")
    lid_collar.visual(
        mesh_from_geometry(_build_lid_body_mesh(), "lid_body"),
        material=lid_black,
        name="lid_body",
    )
    lid_collar.visual(
        mesh_from_geometry(_build_spout_ring_mesh(), "spout_ring"),
        material=dark_plastic,
        name="spout_ring",
    )
    lid_collar.inertial = Inertial.from_geometry(
        Cylinder(radius=0.045, length=0.018),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
    )

    flip_cap = model.part("flip_cap")
    flip_cap.visual(
        Cylinder(radius=0.0036, length=0.014),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lid_black,
        name="hinge_pin",
    )
    flip_cap.visual(
        Box((0.012, 0.018, 0.004)),
        origin=Origin(xyz=(0.004, 0.0, -0.003)),
        material=lid_black,
        name="rear_bridge",
    )
    flip_cap.visual(
        Box((0.052, 0.003, 0.003)),
        origin=Origin(xyz=(0.024, -0.0075, -0.006)),
        material=lid_black,
        name="left_arm",
    )
    flip_cap.visual(
        Box((0.052, 0.003, 0.003)),
        origin=Origin(xyz=(0.024, 0.0075, -0.006)),
        material=lid_black,
        name="right_arm",
    )
    flip_cap.visual(
        Box((0.022, 0.018, 0.004)),
        origin=Origin(xyz=(0.050, 0.0, -0.014)),
        material=lid_black,
        name="cap_cover",
    )
    flip_cap.visual(
        Box((0.016, 0.018, 0.005)),
        origin=Origin(xyz=(0.042, 0.0, -0.010)),
        material=lid_black,
        name="front_bridge",
    )
    flip_cap.visual(
        Box((0.008, 0.010, 0.002)),
        origin=Origin(xyz=(0.061, 0.0, -0.0145)),
        material=lid_black,
        name="thumb_tab",
    )
    flip_cap.visual(
        Cylinder(radius=0.0048, length=0.002),
        origin=Origin(xyz=(0.050, 0.0, -0.0170)),
        material=dark_plastic,
        name="seal_pad",
    )
    flip_cap.inertial = Inertial.from_geometry(
        Box((0.072, 0.048, 0.020)),
        mass=0.04,
        origin=Origin(xyz=(0.004, 0.0, 0.012)),
    )

    model.articulation(
        "base_to_drive",
        ArticulationType.FIXED,
        parent=motor_base,
        child=drive_coupler,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
    )
    model.articulation(
        "base_to_cup",
        ArticulationType.FIXED,
        parent=motor_base,
        child=cup,
        origin=Origin(xyz=(0.0, 0.0, CUP_MOUNT_Z)),
    )
    model.articulation(
        "cup_to_blades",
        ArticulationType.REVOLUTE,
        parent=cup,
        child=blade_assembly,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=30.0,
            lower=0.0,
            upper=math.tau,
        ),
    )
    model.articulation(
        "cup_to_lid",
        ArticulationType.FIXED,
        parent=cup,
        child=lid_collar,
        origin=Origin(xyz=(0.0, 0.0, CUP_HEIGHT)),
    )
    model.articulation(
        "lid_to_flip_cap",
        ArticulationType.REVOLUTE,
        parent=lid_collar,
        child=flip_cap,
        origin=Origin(xyz=(-0.032, 0.0, 0.026)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=5.0,
            lower=-1.9,
            upper=0.0,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    motor_base = object_model.get_part("motor_base")
    drive_coupler = object_model.get_part("drive_coupler")
    cup = object_model.get_part("cup")
    blade_assembly = object_model.get_part("blade_assembly")
    lid_collar = object_model.get_part("lid_collar")
    flip_cap = object_model.get_part("flip_cap")

    cup_to_blades = object_model.get_articulation("cup_to_blades")
    lid_to_flip_cap = object_model.get_articulation("lid_to_flip_cap")

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
    ctx.allow_overlap(
        blade_assembly,
        cup,
        reason="Blade extractor collar intentionally nests over the cup mouth threads.",
    )
    ctx.allow_overlap(
        blade_assembly,
        motor_base,
        reason="Blade carrier nests slightly into the motor base drive recess.",
    )
    ctx.allow_overlap(
        flip_cap,
        lid_collar,
        elem_a="hinge_pin",
        elem_b="lid_body",
        reason="Captured hinge pin passes through the lid hinge knuckle boss.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(drive_coupler, motor_base)
    ctx.expect_contact(blade_assembly, motor_base)
    ctx.expect_contact(blade_assembly, cup)
    ctx.expect_contact(lid_collar, cup)
    ctx.expect_within(cup, motor_base, axes="xy", margin=0.0)
    ctx.expect_overlap(blade_assembly, cup, axes="xy", min_overlap=0.085)

    ctx.check(
        "blade articulation axis is vertical",
        tuple(cup_to_blades.axis) == (0.0, 0.0, 1.0),
        f"Expected (0, 0, 1), got {cup_to_blades.axis!r}",
    )
    blade_limits = cup_to_blades.motion_limits
    ctx.check(
        "blade articulation has broad spin range",
        blade_limits is not None
        and blade_limits.lower is not None
        and blade_limits.upper is not None
        and (blade_limits.upper - blade_limits.lower) >= 6.0,
        f"Unexpected blade limits: {blade_limits!r}",
    )
    ctx.check(
        "flip lid hinge axis is lateral",
        tuple(lid_to_flip_cap.axis) == (0.0, 1.0, 0.0),
        f"Expected (0, 1, 0), got {lid_to_flip_cap.axis!r}",
    )

    with ctx.pose({cup_to_blades: math.pi / 2.0}):
        ctx.expect_contact(blade_assembly, motor_base)
        ctx.expect_contact(blade_assembly, cup)

    with ctx.pose({lid_to_flip_cap: 0.0}):
        ctx.expect_gap(
            flip_cap,
            lid_collar,
            axis="z",
            positive_elem="seal_pad",
            negative_elem="spout_ring",
            max_penetration=0.0012,
            max_gap=0.003,
        )
        ctx.expect_overlap(
            flip_cap,
            lid_collar,
            axes="xy",
            elem_a="seal_pad",
            elem_b="spout_ring",
            min_overlap=0.008,
        )

    seal_rest_aabb = ctx.part_element_world_aabb(flip_cap, elem="seal_pad")
    assert seal_rest_aabb is not None
    with ctx.pose({lid_to_flip_cap: -1.45}):
        ctx.expect_gap(
            flip_cap,
            lid_collar,
            axis="z",
            positive_elem="seal_pad",
            negative_elem="spout_ring",
            min_gap=0.012,
        )
        seal_open_aabb = ctx.part_element_world_aabb(flip_cap, elem="seal_pad")
        assert seal_open_aabb is not None
        seal_rest_center = tuple(
            (seal_rest_aabb[0][i] + seal_rest_aabb[1][i]) * 0.5 for i in range(3)
        )
        seal_open_center = tuple(
            (seal_open_aabb[0][i] + seal_open_aabb[1][i]) * 0.5 for i in range(3)
        )
        ctx.check(
            "flip cap lifts clear of the spout",
            seal_open_center[2] > seal_rest_center[2] + 0.015
            and seal_open_center[0] < seal_rest_center[0] - 0.006,
            f"Rest {seal_rest_center!r} open {seal_open_center!r}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
