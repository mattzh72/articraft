from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="recessed_can_gimbal_spotlight")

    trim_white = model.material("trim_white", rgba=(0.94, 0.95, 0.96, 1.0))
    satin_black = model.material("satin_black", rgba=(0.12, 0.13, 0.14, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.20, 0.21, 0.23, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.75, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.82, 0.88, 0.95, 0.45))

    housing = model.part("housing")
    housing_shell = LatheGeometry.from_shell_profiles(
        [
            (0.105, -0.004),
            (0.105, 0.000),
            (0.087, 0.004),
            (0.086, 0.028),
            (0.086, 0.176),
            (0.088, 0.182),
        ],
        [
            (0.073, -0.004),
            (0.073, 0.016),
            (0.078, 0.028),
            (0.080, 0.176),
            (0.080, 0.182),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    housing.visual(
        _mesh("housing_shell", housing_shell),
        material=trim_white,
        name="trim_housing",
    )
    housing.visual(
        Cylinder(radius=0.089, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.174)),
        material=brushed_aluminum,
        name="mounting_band",
    )

    ring = model.part("rotating_ring")
    ring_shell = LatheGeometry.from_shell_profiles(
        [
            (0.0715, -0.012),
            (0.0730, -0.008),
            (0.0730, 0.010),
            (0.0705, 0.012),
        ],
        [
            (0.0600, -0.012),
            (0.0600, 0.006),
            (0.0620, 0.012),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    ring.visual(
        _mesh("rotating_ring_shell", ring_shell),
        material=satin_black,
        name="ring_shell",
    )
    ring.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.061, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_graphite,
        name="right_bearing_boss",
    )
    ring.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(-0.061, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_graphite,
        name="left_bearing_boss",
    )

    capsule = model.part("lamp_capsule")
    capsule_shell = LatheGeometry.from_shell_profiles(
        [
            (0.012, 0.028),
            (0.024, 0.024),
            (0.037, 0.014),
            (0.046, -0.008),
            (0.052, -0.036),
            (0.054, -0.060),
            (0.050, -0.074),
            (0.043, -0.082),
        ],
        [
            (0.000, 0.026),
            (0.014, 0.022),
            (0.028, 0.014),
            (0.038, -0.008),
            (0.045, -0.036),
            (0.047, -0.060),
            (0.044, -0.074),
            (0.041, -0.082),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    capsule.visual(
        _mesh("lamp_capsule_shell", capsule_shell),
        material=dark_graphite,
        name="capsule_shell",
    )
    capsule.visual(
        Cylinder(radius=0.0415, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.081)),
        material=lens_glass,
        name="front_lens",
    )
    for index, (radius, z_pos) in enumerate(((0.024, 0.022), (0.034, 0.012), (0.041, 0.002))):
        capsule.visual(
            Cylinder(radius=radius, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            material=brushed_aluminum,
            name=f"cooling_fin_{index}",
        )
    capsule.visual(
        Cylinder(radius=0.0045, length=0.012),
        origin=Origin(xyz=(0.049, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_aluminum,
        name="right_trunnion",
    )
    capsule.visual(
        Cylinder(radius=0.0045, length=0.012),
        origin=Origin(xyz=(-0.049, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_aluminum,
        name="left_trunnion",
    )

    model.articulation(
        "housing_to_ring",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=ring,
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=4.0),
    )
    model.articulation(
        "ring_to_capsule",
        ArticulationType.REVOLUTE,
        parent=ring,
        child=capsule,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=-0.82,
            upper=0.82,
        ),
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
    ring = object_model.get_part("rotating_ring")
    capsule = object_model.get_part("lamp_capsule")
    spin = object_model.get_articulation("housing_to_ring")
    tilt = object_model.get_articulation("ring_to_capsule")

    ctx.check(
        "all spotlight parts exist",
        housing is not None and ring is not None and capsule is not None,
        details=f"housing={housing}, ring={ring}, capsule={capsule}",
    )
    ctx.check(
        "ring rotates continuously about vertical axis",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.check(
        "lamp capsule tilts on a horizontal equator pivot",
        tilt.articulation_type == ArticulationType.REVOLUTE
        and tuple(tilt.axis) == (1.0, 0.0, 0.0)
        and tilt.motion_limits is not None
        and tilt.motion_limits.lower is not None
        and tilt.motion_limits.upper is not None
        and tilt.motion_limits.lower < 0.0 < tilt.motion_limits.upper,
        details=f"type={tilt.articulation_type}, axis={tilt.axis}, limits={tilt.motion_limits}",
    )

    with ctx.pose({spin: 0.0, tilt: 0.0}):
        ctx.expect_within(
            ring,
            housing,
            axes="xy",
            inner_elem="ring_shell",
            outer_elem="trim_housing",
            margin=0.002,
            name="rotating ring stays nested inside the ceiling trim opening",
        )
        ctx.expect_overlap(
            capsule,
            ring,
            axes="xy",
            elem_a="capsule_shell",
            elem_b="ring_shell",
            min_overlap=0.080,
            name="lamp capsule stays centered within the gimbal ring at rest",
        )
        ctx.expect_contact(
            capsule,
            ring,
            elem_a="left_trunnion",
            elem_b="left_bearing_boss",
            contact_tol=1e-4,
            name="left trunnion meets left bearing boss",
        )
        ctx.expect_contact(
            capsule,
            ring,
            elem_a="right_trunnion",
            elem_b="right_bearing_boss",
            contact_tol=1e-4,
            name="right trunnion meets right bearing boss",
        )

        rest_aabb = ctx.part_element_world_aabb(capsule, elem="capsule_shell")

    with ctx.pose({tilt: 0.55}):
        tilted_aabb = ctx.part_element_world_aabb(capsule, elem="capsule_shell")

    def _center_y(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][1] + aabb[1][1])

    rest_center_y = _center_y(rest_aabb)
    tilted_center_y = _center_y(tilted_aabb)
    ctx.check(
        "positive tilt swings the lamp capsule off vertical",
        rest_center_y is not None
        and tilted_center_y is not None
        and tilted_center_y > rest_center_y + 0.01,
        details=f"rest_center_y={rest_center_y}, tilted_center_y={tilted_center_y}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
