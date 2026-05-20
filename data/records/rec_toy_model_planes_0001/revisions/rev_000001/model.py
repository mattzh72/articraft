from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    BoxGeometry,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    LoftGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _airfoil_loop(
    x_front: float, chord: float, thickness: float, z_center: float
) -> list[tuple[float, float]]:
    return [
        (x_front - 0.00 * chord, z_center),
        (x_front - 0.04 * chord, z_center + 0.46 * thickness),
        (x_front - 0.18 * chord, z_center + 0.62 * thickness),
        (x_front - 0.44 * chord, z_center + 0.44 * thickness),
        (x_front - 0.78 * chord, z_center + 0.15 * thickness),
        (x_front - 1.00 * chord, z_center + 0.02 * thickness),
        (x_front - 0.90 * chord, z_center - 0.05 * thickness),
        (x_front - 0.58 * chord, z_center - 0.18 * thickness),
        (x_front - 0.24 * chord, z_center - 0.24 * thickness),
        (x_front - 0.06 * chord, z_center - 0.11 * thickness),
    ]


def _section_profile(
    span_y: float, x_front: float, chord: float, thickness: float, z_center: float
) -> list[tuple[float, float, float]]:
    return [(x, z, span_y) for x, z in _airfoil_loop(x_front, chord, thickness, z_center)]


def _build_airframe_meshes() -> dict[str, object]:
    fuselage_profile = [
        (0.000, -0.158),
        (0.012, -0.148),
        (0.020, -0.130),
        (0.028, -0.092),
        (0.033, -0.040),
        (0.036, 0.012),
        (0.034, 0.060),
        (0.029, 0.105),
        (0.018, 0.136),
        (0.000, 0.152),
    ]
    fuselage_geom = LatheGeometry(fuselage_profile, segments=48)
    fuselage_geom.rotate_y(math.pi / 2.0)

    wing_profiles = []
    for y in (-0.175, -0.090, 0.0, 0.090, 0.175):
        ratio = abs(y) / 0.175
        wing_profiles.append(
            _section_profile(
                span_y=y,
                x_front=0.045 - 0.018 * ratio,
                chord=0.172 - 0.098 * ratio,
                thickness=0.018 - 0.007 * ratio,
                z_center=0.014 * ratio,
            )
        )
    wing_geom = LoftGeometry(wing_profiles, cap=True, closed=True)
    wing_geom.rotate_x(math.pi / 2.0)

    tail_profiles = []
    for y in (-0.088, -0.040, 0.0, 0.040, 0.088):
        ratio = abs(y) / 0.088
        tail_profiles.append(
            _section_profile(
                span_y=y,
                x_front=-0.084 - 0.010 * ratio,
                chord=0.078 - 0.026 * ratio,
                thickness=0.008 - 0.003 * ratio,
                z_center=0.018 + 0.003 * ratio,
            )
        )
    tail_geom = LoftGeometry(tail_profiles, cap=True, closed=True)
    tail_geom.rotate_x(math.pi / 2.0)

    fin_profile = [
        (-0.160, 0.015),
        (-0.144, 0.020),
        (-0.118, 0.052),
        (-0.112, 0.070),
        (-0.138, 0.078),
        (-0.156, 0.050),
    ]
    fin_geom = ExtrudeGeometry.centered(fin_profile, 0.012, cap=True, closed=True)
    fin_geom.rotate_x(math.pi / 2.0)

    canopy_geom = SphereGeometry(1.0, width_segments=28, height_segments=18)
    canopy_geom.scale(0.046, 0.031, 0.023).translate(0.010, 0.0, 0.026)

    left_main_strut = tube_from_spline_points(
        [(0.008, -0.040, -0.004), (0.046, -0.074, -0.052)],
        radius=0.0045,
        samples_per_segment=6,
        radial_segments=16,
    )
    right_main_strut = tube_from_spline_points(
        [(0.008, 0.040, -0.004), (0.046, 0.074, -0.052)],
        radius=0.0045,
        samples_per_segment=6,
        radial_segments=16,
    )
    tail_strut = tube_from_spline_points(
        [(-0.126, 0.0, -0.004), (-0.146, 0.0, -0.038)],
        radius=0.003,
        samples_per_segment=6,
        radial_segments=14,
    )

    return {
        "fuselage": _save_mesh("toy_plane_fuselage.obj", fuselage_geom),
        "wing": _save_mesh("toy_plane_wing.obj", wing_geom),
        "tailplane": _save_mesh("toy_plane_tailplane.obj", tail_geom),
        "fin": _save_mesh("toy_plane_fin.obj", fin_geom),
        "canopy": _save_mesh("toy_plane_canopy.obj", canopy_geom),
        "left_main_strut": _save_mesh("toy_plane_left_main_strut.obj", left_main_strut),
        "right_main_strut": _save_mesh("toy_plane_right_main_strut.obj", right_main_strut),
        "tail_strut": _save_mesh("toy_plane_tail_strut.obj", tail_strut),
    }


def _build_propeller_mesh():
    propeller_geom = CylinderGeometry(radius=0.010, height=0.014, radial_segments=28, closed=True)
    propeller_geom.rotate_y(math.pi / 2.0).translate(0.015, 0.0, 0.0)

    spinner_geom = ConeGeometry(radius=0.018, height=0.030, radial_segments=32, closed=True)
    spinner_geom.rotate_y(math.pi / 2.0).translate(0.028, 0.0, 0.0)
    propeller_geom.merge(spinner_geom)

    blade_proto = BoxGeometry((0.0045, 0.022, 0.118))
    blade_proto.rotate_y(0.28).translate(0.020, 0.0, 0.060)
    for blade_index in range(3):
        propeller_geom.merge(blade_proto.copy().rotate_x((2.0 * math.pi * blade_index) / 3.0))

    cuff_proto = BoxGeometry((0.006, 0.018, 0.034))
    cuff_proto.rotate_y(0.28).translate(0.016, 0.0, 0.018)
    for blade_index in range(3):
        propeller_geom.merge(cuff_proto.copy().rotate_x((2.0 * math.pi * blade_index) / 3.0))

    return _save_mesh("toy_plane_propeller.obj", propeller_geom)


def _aabb_bounds(aabb) -> tuple[float, float, float, float, float, float]:
    if all(hasattr(aabb, name) for name in ("min_x", "max_x", "min_y", "max_y", "min_z", "max_z")):
        return (aabb.min_x, aabb.max_x, aabb.min_y, aabb.max_y, aabb.min_z, aabb.max_z)
    if hasattr(aabb, "mins") and hasattr(aabb, "maxs"):
        mins = tuple(aabb.mins)
        maxs = tuple(aabb.maxs)
        return (mins[0], maxs[0], mins[1], maxs[1], mins[2], maxs[2])
    if hasattr(aabb, "min") and hasattr(aabb, "max"):
        mins = tuple(aabb.min)
        maxs = tuple(aabb.max)
        return (mins[0], maxs[0], mins[1], maxs[1], mins[2], maxs[2])
    if isinstance(aabb, (tuple, list)) and len(aabb) == 2:
        mins = tuple(aabb[0])
        maxs = tuple(aabb[1])
        return (mins[0], maxs[0], mins[1], maxs[1], mins[2], maxs[2])
    raise AssertionError(f"Unsupported AABB value: {aabb!r}")


def _assert_between(label: str, value: float, lower: float, upper: float) -> None:
    if not (lower <= value <= upper):
        raise AssertionError(f"{label}={value:.4f} outside [{lower:.4f}, {upper:.4f}]")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="collectible_toy_airplane", assets=ASSETS)
    airframe_meshes = _build_airframe_meshes()
    propeller_mesh = _build_propeller_mesh()

    airframe = model.part("airframe")
    airframe.visual(airframe_meshes["fuselage"], origin=Origin(xyz=(0.0, 0.0, 0.0)))
    airframe.visual(airframe_meshes["wing"], origin=Origin(xyz=(0.0, 0.0, 0.0)))
    airframe.visual(airframe_meshes["tailplane"], origin=Origin(xyz=(0.0, 0.0, 0.0)))
    airframe.visual(airframe_meshes["fin"], origin=Origin(xyz=(0.0, 0.0, 0.0)))
    airframe.visual(airframe_meshes["canopy"], origin=Origin(xyz=(0.0, 0.0, 0.0)))
    airframe.visual(airframe_meshes["left_main_strut"], origin=Origin(xyz=(0.0, 0.0, 0.0)))
    airframe.visual(airframe_meshes["right_main_strut"], origin=Origin(xyz=(0.0, 0.0, 0.0)))
    airframe.visual(airframe_meshes["tail_strut"], origin=Origin(xyz=(0.0, 0.0, 0.0)))
    airframe.visual(
        Cylinder(radius=0.019, length=0.012),
        origin=Origin(xyz=(0.046, -0.074, -0.052), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    airframe.visual(
        Cylinder(radius=0.019, length=0.012),
        origin=Origin(xyz=(0.046, 0.074, -0.052), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    airframe.visual(
        Cylinder(radius=0.011, length=0.008),
        origin=Origin(xyz=(-0.146, 0.0, -0.040), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    airframe.visual(
        Cylinder(radius=0.003, length=0.018),
        origin=Origin(xyz=(0.090, -0.024, 0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    airframe.visual(
        Cylinder(radius=0.003, length=0.018),
        origin=Origin(xyz=(0.090, 0.024, 0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    airframe.inertial = Inertial.from_geometry(
        Box((0.330, 0.360, 0.120)),
        mass=0.36,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )

    propeller = model.part("propeller")
    propeller.visual(propeller_mesh, origin=Origin(xyz=(0.0, 0.0, 0.0)))
    propeller.inertial = Inertial.from_geometry(
        Box((0.070, 0.230, 0.230)),
        mass=0.03,
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
    )

    model.articulation(
        "propeller_spin",
        ArticulationType.CONTINUOUS,
        parent="airframe",
        child="propeller",
        origin=Origin(xyz=(0.154, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.003, overlap_volume_tol=0.0)

    airframe_bounds = _aabb_bounds(ctx.part_world_aabb("airframe"))
    airframe_length = airframe_bounds[1] - airframe_bounds[0]
    airframe_span = airframe_bounds[3] - airframe_bounds[2]
    airframe_height = airframe_bounds[5] - airframe_bounds[4]
    _assert_between("airframe length", airframe_length, 0.29, 0.36)
    _assert_between("airframe wingspan", airframe_span, 0.30, 0.40)
    _assert_between("airframe height", airframe_height, 0.09, 0.16)
    if airframe_bounds[4] > -0.030:
        raise AssertionError("Landing gear should extend visibly below the fuselage.")
    if airframe_bounds[5] < 0.070:
        raise AssertionError("Canopy and tail fin should create a tall airplane silhouette.")
    if airframe_span < airframe_length * 0.95:
        raise AssertionError("Wings should read clearly in the collectible airplane silhouette.")

    rest_prop_bounds = _aabb_bounds(ctx.part_world_aabb("propeller"))
    propeller_gap = rest_prop_bounds[0] - airframe_bounds[1]
    propeller_span_y = rest_prop_bounds[3] - rest_prop_bounds[2]
    propeller_span_z = rest_prop_bounds[5] - rest_prop_bounds[4]
    _assert_between("propeller nose gap", propeller_gap, 0.002, 0.020)
    _assert_between("propeller span y", propeller_span_y, 0.17, 0.24)
    _assert_between("propeller span z", propeller_span_z, 0.17, 0.24)

    reference_position = tuple(ctx.part_world_position("propeller"))
    if abs(reference_position[1]) > 0.004 or abs(reference_position[2]) > 0.004:
        raise AssertionError("Propeller should remain centered on the fuselage nose.")

    for angle in (0.0, 1.1, 2.2):
        with ctx.pose(propeller_spin=angle):
            posed_airframe_bounds = _aabb_bounds(ctx.part_world_aabb("airframe"))
            posed_prop_bounds = _aabb_bounds(ctx.part_world_aabb("propeller"))
            posed_gap = posed_prop_bounds[0] - posed_airframe_bounds[1]
            _assert_between(f"propeller nose gap at {angle:.1f} rad", posed_gap, 0.002, 0.020)

            posed_position = tuple(ctx.part_world_position("propeller"))
            if any(abs(a - b) > 1e-4 for a, b in zip(reference_position, posed_position)):
                raise AssertionError(
                    "Propeller rotation should spin in place without drifting off-axis."
                )

            posed_span_y = posed_prop_bounds[3] - posed_prop_bounds[2]
            posed_span_z = posed_prop_bounds[5] - posed_prop_bounds[4]
            if min(posed_span_y, posed_span_z) < 0.15:
                raise AssertionError(
                    "Propeller should keep a substantial collectible silhouette while spinning."
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
