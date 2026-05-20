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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_tangent_panel(
    part,
    *,
    name: str,
    radius: float,
    angle_deg: float,
    size: tuple[float, float, float],
    z: float,
    material,
) -> None:
    angle = math.radians(angle_deg)
    part.visual(
        Box(size),
        origin=Origin(
            xyz=(radius * math.cos(angle), radius * math.sin(angle), z),
            rpy=(0.0, 0.0, angle + math.pi / 2.0),
        ),
        material=material,
        name=name,
    )


def _add_wing(part, *, prefix: str, angle_deg: float, glass_material, frame_material) -> None:
    yaw = math.radians(angle_deg)
    c = math.cos(yaw)
    s = math.sin(yaw)

    def radial_pose(x: float, z: float) -> Origin:
        return Origin(xyz=(x * c, x * s, z), rpy=(0.0, 0.0, yaw))

    part.visual(
        Box((0.95, 0.038, 2.10)),
        origin=radial_pose(0.535, 1.125),
        material=glass_material,
        name=f"{prefix}_glass",
    )
    part.visual(
        Box((0.06, 0.055, 2.18)),
        origin=radial_pose(1.01, 1.13),
        material=frame_material,
        name=f"{prefix}_outer_frame",
    )
    part.visual(
        Box((1.00, 0.055, 0.06)),
        origin=radial_pose(0.54, 2.155),
        material=frame_material,
        name=f"{prefix}_top_rail",
    )
    part.visual(
        Box((1.00, 0.055, 0.06)),
        origin=radial_pose(0.54, 0.095),
        material=frame_material,
        name=f"{prefix}_bottom_rail",
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))


def _circle_profile(radius: float, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cleanroom_revolving_door")

    stainless = model.material("stainless", rgba=(0.73, 0.75, 0.77, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.33, 0.35, 0.38, 1.0))
    housing_glass = model.material("housing_glass", rgba=(0.78, 0.88, 0.94, 0.22))
    wing_glass = model.material("wing_glass", rgba=(0.72, 0.84, 0.92, 0.34))

    housing = model.part("housing")
    housing.visual(
        Cylinder(radius=1.26, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 2.41)),
        material=stainless,
        name="canopy",
    )
    housing.visual(
        Cylinder(radius=1.30, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 2.505)),
        material=dark_steel,
        name="canopy_cap",
    )
    housing.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                _circle_profile(1.26),
                [_circle_profile(0.20)],
                0.04,
                cap=True,
                center=True,
            ),
            "floor_plenum_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=dark_steel,
        name="floor_plenum",
    )
    housing.visual(
        Cylinder(radius=0.22, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 2.365)),
        material=dark_steel,
        name="ceiling_bearing_cover",
    )

    for index, angle_deg in enumerate((36, 60, 84, 108, 132, 216, 240, 264, 288, 312)):
        _add_tangent_panel(
            housing,
            name=f"drum_glass_{index:02d}",
            radius=1.17,
            angle_deg=angle_deg,
            size=(0.46, 0.04, 2.32),
            z=1.18,
            material=housing_glass,
        )

    for index, angle_deg in enumerate((24, 156, 204, 336)):
        _add_tangent_panel(
            housing,
            name=f"opening_jamb_{index:02d}",
            radius=1.17,
            angle_deg=angle_deg,
            size=(0.12, 0.08, 2.34),
            z=1.17,
            material=stainless,
        )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.075, length=2.24),
        origin=Origin(xyz=(0.0, 0.0, 1.15)),
        material=stainless,
        name="central_post",
    )
    rotor.visual(
        Cylinder(radius=0.14, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=dark_steel,
        name="bottom_hub",
    )
    rotor.visual(
        Cylinder(radius=0.14, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 2.18)),
        material=dark_steel,
        name="top_hub",
    )

    _add_wing(rotor, prefix="wing_a", angle_deg=0.0, glass_material=wing_glass, frame_material=stainless)
    _add_wing(rotor, prefix="wing_b", angle_deg=120.0, glass_material=wing_glass, frame_material=stainless)
    _add_wing(rotor, prefix="wing_c", angle_deg=240.0, glass_material=wing_glass, frame_material=stainless)

    model.articulation(
        "housing_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=rotor,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.9),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    housing = object_model.get_part("housing")
    rotor = object_model.get_part("rotor")
    rotation = object_model.get_articulation("housing_to_rotor")

    wing_a_glass = rotor.get_visual("wing_a_glass")
    wing_b_glass = rotor.get_visual("wing_b_glass")
    wing_c_glass = rotor.get_visual("wing_c_glass")
    canopy = housing.get_visual("canopy")
    floor = housing.get_visual("floor_plenum")

    ctx.check(
        "hospital revolving door parts exist",
        housing is not None and rotor is not None and rotation is not None,
        details=f"housing={housing}, rotor={rotor}, rotation={rotation}",
    )
    ctx.check(
        "three sealed wings are present",
        wing_a_glass is not None and wing_b_glass is not None and wing_c_glass is not None,
        details=f"wing_a={wing_a_glass}, wing_b={wing_b_glass}, wing_c={wing_c_glass}",
    )
    ctx.check(
        "rotor uses continuous vertical rotation",
        rotation.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in rotation.axis) == (0.0, 0.0, 1.0)
        and rotation.motion_limits is not None
        and rotation.motion_limits.lower is None
        and rotation.motion_limits.upper is None,
        details=(
            f"type={rotation.articulation_type}, axis={rotation.axis}, "
            f"limits={rotation.motion_limits}"
        ),
    )
    ctx.expect_origin_distance(
        rotor,
        housing,
        axes="xy",
        max_dist=0.001,
        name="rotor is centered on the drum axis",
    )

    with ctx.pose({rotation: 0.0}):
        ctx.expect_gap(
            rotor,
            housing,
            axis="z",
            positive_elem=wing_a_glass,
            negative_elem=floor,
            min_gap=0.03,
            max_gap=0.08,
            name="wing clears the sealed floor plenum",
        )
        ctx.expect_gap(
            housing,
            rotor,
            axis="z",
            positive_elem=canopy,
            negative_elem=wing_a_glass,
            min_gap=0.10,
            max_gap=0.20,
            name="wing clears the sealed canopy drum",
        )

    with ctx.pose({rotation: 1.0}):
        wing_a_aabb = ctx.part_element_world_aabb(rotor, elem=wing_a_glass)
        wing_a_center = _aabb_center(wing_a_aabb)
        ctx.check(
            "positive rotation swings the lead wing toward +y",
            wing_a_center is not None
            and wing_a_center[1] > 0.38
            and wing_a_center[0] < 0.40,
            details=f"wing_a_center={wing_a_center}",
        )
        ctx.expect_gap(
            rotor,
            housing,
            axis="z",
            positive_elem=wing_a_glass,
            negative_elem=floor,
            min_gap=0.03,
            max_gap=0.08,
            name="rotating wing still clears the floor plenum",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
