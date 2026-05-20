from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _circle_profile(
    center: tuple[float, float],
    radius: float,
    *,
    segments: int = 18,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * index) / segments),
            cy + radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _profile_yz_to_extrude_plane(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return [(-z, y) for y, z in points]


def _extrude_yz_profile(
    outer_profile: list[tuple[float, float]],
    *,
    hole_profiles: list[list[tuple[float, float]]] | None = None,
    thickness: float,
):
    geometry = ExtrudeWithHolesGeometry(
        _profile_yz_to_extrude_plane(outer_profile),
        [
            _profile_yz_to_extrude_plane(hole_profile)
            for hole_profile in (hole_profiles or [])
        ],
        height=thickness,
        center=False,
    )
    return geometry.rotate_y(math.pi / 2.0)


def _crank_arm_outline() -> list[tuple[float, float]]:
    return [
        (0.018, 0.018),
        (0.021, 0.026),
        (0.018, 0.046),
        (0.014, 0.082),
        (0.0125, 0.118),
        (0.014, 0.146),
        (0.012, 0.160),
        (0.000, 0.168),
        (-0.012, 0.160),
        (-0.014, 0.146),
        (-0.0125, 0.118),
        (-0.014, 0.082),
        (-0.018, 0.046),
        (-0.021, 0.026),
        (-0.018, 0.018),
    ]


def _build_crank_arm_mesh():
    holes = [
        _circle_profile((0.0, 0.054), 0.0048, segments=18),
        _circle_profile((0.0, 0.090), 0.0063, segments=20),
        _circle_profile((0.0, 0.126), 0.0052, segments=18),
    ]
    return _extrude_yz_profile(_crank_arm_outline(), hole_profiles=holes, thickness=0.014)


def _build_chainring_mesh():
    tooth_count = 30
    root_radius = 0.087
    tip_radius = 0.096
    outer_profile: list[tuple[float, float]] = []
    for index in range(tooth_count * 2):
        angle = (math.pi * index) / tooth_count
        radius = tip_radius if index % 2 == 0 else root_radius
        outer_profile.append((radius * math.cos(angle), radius * math.sin(angle)))

    hole_profiles = [_circle_profile((0.0, 0.0), 0.0145, segments=24)]
    for bolt_index in range(5):
        angle = (2.0 * math.pi * bolt_index) / 5.0 + math.pi / 10.0
        hole_profiles.append(
            _circle_profile(
                (0.024 * math.cos(angle), 0.024 * math.sin(angle)),
                0.0045,
                segments=18,
            )
        )
    return _extrude_yz_profile(outer_profile, hole_profiles=hole_profiles, thickness=0.004)


def _add_crank_visuals(part, *, crank_material, hardware_material, has_chainring_mount: bool) -> None:
    arm_mesh = _save_mesh("bmx_crank_arm.obj", _build_crank_arm_mesh())
    part.visual(arm_mesh, material=crank_material, name="arm_plate")
    part.visual(
        Cylinder(radius=0.021, length=0.020),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=crank_material,
        name="spindle_boss",
    )
    part.visual(
        Box((0.020, 0.038, 0.030)),
        origin=Origin(xyz=(0.010, 0.0, 0.010)),
        material=crank_material,
        name="clamp_block",
    )
    part.visual(
        Cylinder(radius=0.0036, length=0.048),
        origin=Origin(xyz=(0.013, 0.0, 0.007), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_material,
        name="pinch_bolt",
    )
    part.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.170), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=crank_material,
        name="pedal_eye",
    )
    part.visual(
        Box((0.016, 0.028, 0.020)),
        origin=Origin(xyz=(0.008, 0.0, 0.158)),
        material=crank_material,
        name="pedal_pad",
    )
    if has_chainring_mount:
        part.visual(
            Cylinder(radius=0.033, length=0.004),
            origin=Origin(xyz=(0.016, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hardware_material,
            name="chainring_mount",
        )
    part.inertial = Inertial.from_geometry(
        Box((0.024, 0.046, 0.182)),
        mass=0.34,
        origin=Origin(xyz=(0.012, 0.0, 0.090)),
    )


def _add_pedal_spindle_visuals(part, *, metal_material) -> None:
    part.visual(
        Cylinder(radius=0.0048, length=0.014),
        origin=Origin(xyz=(0.007, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_material,
        name="threaded_stub",
    )
    part.visual(
        Cylinder(radius=0.0068, length=0.006),
        origin=Origin(xyz=(0.017, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_material,
        name="shoulder",
    )
    part.visual(
        Cylinder(radius=0.0041, length=0.028),
        origin=Origin(xyz=(0.032, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_material,
        name="axle",
    )
    part.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0068, length=0.040),
        mass=0.07,
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )


def _add_pedal_body_visuals(part, *, body_material, stud_material) -> None:
    part.visual(
        Cylinder(radius=0.0092, length=0.036),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_material,
        name="axle_barrel",
    )
    part.visual(
        Box((0.018, 0.096, 0.014)),
        origin=Origin(),
        material=body_material,
        name="platform_core",
    )
    part.visual(
        Box((0.024, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, 0.043, 0.001)),
        material=body_material,
        name="front_rail",
    )
    part.visual(
        Box((0.024, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -0.043, 0.001)),
        material=body_material,
        name="rear_rail",
    )
    part.visual(
        Box((0.024, 0.080, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=body_material,
        name="top_rib",
    )
    part.visual(
        Box((0.024, 0.080, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=body_material,
        name="bottom_rib",
    )

    stud_index = 0
    for stud_x in (-0.0065, 0.0065):
        for stud_y in (-0.032, 0.0, 0.032):
            for stud_z in (-0.009, 0.009):
                part.visual(
                    Cylinder(radius=0.0018, length=0.004),
                    origin=Origin(
                        xyz=(stud_x, stud_y, stud_z),
                        rpy=(0.0, 0.0, 0.0) if stud_z > 0.0 else (math.pi, 0.0, 0.0),
                    ),
                    material=stud_material,
                    name=f"stud_{stud_index}",
                )
                stud_index += 1

    part.inertial = Inertial.from_geometry(
        Box((0.026, 0.100, 0.024)),
        mass=0.16,
        origin=Origin(),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bmx_three_piece_crankset", assets=ASSETS)

    black_oxide = model.material("black_oxide", rgba=(0.12, 0.12, 0.13, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.27, 0.28, 0.31, 1.0))
    machined_alloy = model.material("machined_alloy", rgba=(0.72, 0.74, 0.77, 1.0))
    bolt_steel = model.material("bolt_steel", rgba=(0.60, 0.62, 0.66, 1.0))

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.0095, length=0.140),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="spindle_core",
    )
    spindle.visual(
        Cylinder(radius=0.0125, length=0.036),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="center_sleeve",
    )
    spindle.visual(
        Box((0.018, 0.016, 0.016)),
        origin=Origin(xyz=(-0.079, 0.0, 0.0)),
        material=dark_steel,
        name="left_taper",
    )
    spindle.visual(
        Box((0.018, 0.016, 0.016)),
        origin=Origin(xyz=(0.079, 0.0, 0.0)),
        material=dark_steel,
        name="right_taper",
    )
    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.013, length=0.176),
        mass=0.42,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    left_crank = model.part("left_crank")
    _add_crank_visuals(
        left_crank,
        crank_material=black_oxide,
        hardware_material=bolt_steel,
        has_chainring_mount=False,
    )

    right_crank = model.part("right_crank")
    _add_crank_visuals(
        right_crank,
        crank_material=black_oxide,
        hardware_material=bolt_steel,
        has_chainring_mount=True,
    )

    chainring = model.part("chainring")
    chainring.visual(
        _save_mesh("bmx_chainring.obj", _build_chainring_mesh()),
        material=dark_steel,
        name="ring_body",
    )
    for bolt_index in range(5):
        angle = (2.0 * math.pi * bolt_index) / 5.0 + math.pi / 10.0
        chainring.visual(
            Cylinder(radius=0.0048, length=0.010),
            origin=Origin(
                xyz=(0.001, 0.024 * math.cos(angle), 0.024 * math.sin(angle)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=bolt_steel,
            name=f"bolt_{bolt_index}",
        )
    chainring.inertial = Inertial.from_geometry(
        Box((0.012, 0.194, 0.194)),
        mass=0.18,
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
    )

    left_pedal_spindle = model.part("left_pedal_spindle")
    _add_pedal_spindle_visuals(left_pedal_spindle, metal_material=bolt_steel)

    right_pedal_spindle = model.part("right_pedal_spindle")
    _add_pedal_spindle_visuals(right_pedal_spindle, metal_material=bolt_steel)

    left_pedal = model.part("left_pedal")
    _add_pedal_body_visuals(left_pedal, body_material=machined_alloy, stud_material=bolt_steel)

    right_pedal = model.part("right_pedal")
    _add_pedal_body_visuals(right_pedal, body_material=machined_alloy, stud_material=bolt_steel)

    model.articulation(
        "spindle_to_left_crank",
        ArticulationType.FIXED,
        parent=spindle,
        child=left_crank,
        origin=Origin(xyz=(-0.088, 0.0, 0.0), rpy=(0.0, 0.0, math.pi)),
    )
    model.articulation(
        "spindle_to_right_crank",
        ArticulationType.FIXED,
        parent=spindle,
        child=right_crank,
        origin=Origin(xyz=(0.088, 0.0, 0.0), rpy=(math.pi, 0.0, 0.0)),
    )
    model.articulation(
        "right_crank_to_chainring",
        ArticulationType.FIXED,
        parent=right_crank,
        child=chainring,
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
    )
    model.articulation(
        "left_crank_to_pedal_spindle",
        ArticulationType.FIXED,
        parent=left_crank,
        child=left_pedal_spindle,
        origin=Origin(xyz=(0.018, 0.0, 0.170)),
    )
    model.articulation(
        "right_crank_to_pedal_spindle",
        ArticulationType.FIXED,
        parent=right_crank,
        child=right_pedal_spindle,
        origin=Origin(xyz=(0.018, 0.0, 0.170)),
    )
    model.articulation(
        "left_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=left_pedal_spindle,
        child=left_pedal,
        origin=Origin(xyz=(0.032, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=18.0),
    )
    model.articulation(
        "right_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=right_pedal_spindle,
        child=right_pedal,
        origin=Origin(xyz=(0.032, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    spindle = object_model.get_part("spindle")
    left_crank = object_model.get_part("left_crank")
    right_crank = object_model.get_part("right_crank")
    chainring = object_model.get_part("chainring")
    left_pedal_spindle = object_model.get_part("left_pedal_spindle")
    right_pedal_spindle = object_model.get_part("right_pedal_spindle")
    left_pedal = object_model.get_part("left_pedal")
    right_pedal = object_model.get_part("right_pedal")

    left_pedal_spin = object_model.get_articulation("left_pedal_spin")
    right_pedal_spin = object_model.get_articulation("right_pedal_spin")

    left_taper = spindle.get_visual("left_taper")
    right_taper = spindle.get_visual("right_taper")
    left_boss = left_crank.get_visual("spindle_boss")
    right_boss = right_crank.get_visual("spindle_boss")
    left_pinch_bolt = left_crank.get_visual("pinch_bolt")
    right_pinch_bolt = right_crank.get_visual("pinch_bolt")
    left_pedal_eye = left_crank.get_visual("pedal_eye")
    right_pedal_eye = right_crank.get_visual("pedal_eye")
    chainring_mount = right_crank.get_visual("chainring_mount")
    ring_body = chainring.get_visual("ring_body")
    first_chainring_bolt = chainring.get_visual("bolt_0")
    left_thread = left_pedal_spindle.get_visual("threaded_stub")
    right_thread = right_pedal_spindle.get_visual("threaded_stub")
    left_axle = left_pedal_spindle.get_visual("axle")
    right_axle = right_pedal_spindle.get_visual("axle")
    left_barrel = left_pedal.get_visual("axle_barrel")
    right_barrel = right_pedal.get_visual("axle_barrel")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(
        left_pedal,
        left_pedal_spindle,
        reason="Pedal body rotates around its captured spindle axle and barrel bushing.",
    )
    ctx.allow_overlap(
        right_pedal,
        right_pedal_spindle,
        reason="Pedal body rotates around its captured spindle axle and barrel bushing.",
    )

    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_gap(
        spindle,
        left_crank,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=left_taper,
        negative_elem=left_boss,
    )
    ctx.expect_gap(
        right_crank,
        spindle,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=right_boss,
        negative_elem=right_taper,
    )
    ctx.expect_overlap(
        left_crank,
        spindle,
        axes="yz",
        min_overlap=0.004,
        elem_a=left_pinch_bolt,
        elem_b=left_taper,
    )
    ctx.expect_overlap(
        right_crank,
        spindle,
        axes="yz",
        min_overlap=0.004,
        elem_a=right_pinch_bolt,
        elem_b=right_taper,
    )
    ctx.expect_gap(
        left_crank,
        left_pedal_spindle,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=left_pedal_eye,
        negative_elem=left_thread,
    )
    ctx.expect_gap(
        right_pedal_spindle,
        right_crank,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=right_thread,
        negative_elem=right_pedal_eye,
    )
    ctx.expect_gap(
        chainring,
        right_crank,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0005,
        positive_elem=ring_body,
        negative_elem=chainring_mount,
    )
    ctx.expect_contact(chainring, right_crank, elem_a=first_chainring_bolt, elem_b=chainring_mount)
    ctx.expect_overlap(
        chainring,
        right_crank,
        axes="yz",
        min_overlap=0.050,
        elem_a=ring_body,
        elem_b=chainring_mount,
    )
    ctx.expect_overlap(chainring, spindle, axes="yz", min_overlap=0.025)
    ctx.expect_gap(
        left_crank,
        right_crank,
        axis="z",
        min_gap=0.30,
        positive_elem=left_pedal_eye,
        negative_elem=right_pedal_eye,
    )
    ctx.expect_overlap(
        left_pedal,
        left_pedal_spindle,
        axes="yz",
        min_overlap=0.008,
        elem_a=left_barrel,
        elem_b=left_axle,
    )
    ctx.expect_overlap(
        right_pedal,
        right_pedal_spindle,
        axes="yz",
        min_overlap=0.008,
        elem_a=right_barrel,
        elem_b=right_axle,
    )
    with ctx.pose({left_pedal_spin: math.pi / 2.0, right_pedal_spin: -math.pi / 3.0}):
        ctx.expect_overlap(
            left_pedal,
            left_pedal_spindle,
            axes="yz",
            min_overlap=0.008,
            elem_a=left_barrel,
            elem_b=left_axle,
        )
        ctx.expect_overlap(
            right_pedal,
            right_pedal_spindle,
            axes="yz",
            min_overlap=0.008,
            elem_a=right_barrel,
            elem_b=right_axle,
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
