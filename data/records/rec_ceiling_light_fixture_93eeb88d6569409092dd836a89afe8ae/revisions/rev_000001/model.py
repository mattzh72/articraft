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


def _build_canopy_shell():
    return mesh_from_geometry(
        LatheGeometry(
            [
                (0.0, 0.000),
                (0.016, 0.000),
                (0.038, -0.010),
                (0.058, -0.024),
                (0.046, -0.038),
                (0.0, -0.038),
            ],
            segments=64,
        ),
        "ceiling_canopy_shell",
    )


def _build_diffuser_ring_shell():
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.158, -0.012),
                (0.158, -0.028),
            ],
            [
                (0.110, -0.014),
                (0.110, -0.026),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        "diffuser_ring_shell",
    )


def _build_shade_shell():
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.048, -0.040),
                (0.070, -0.068),
                (0.122, -0.140),
                (0.205, -0.238),
            ],
            [
                (0.042, -0.040),
                (0.064, -0.068),
                (0.115, -0.140),
                (0.198, -0.238),
            ],
            segments=80,
            start_cap="flat",
            end_cap="flat",
        ),
        "conical_shade_shell",
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[index] + upper[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="conical_pendant_light")

    canopy_metal = model.material("canopy_metal", rgba=(0.28, 0.29, 0.31, 1.0))
    rod_metal = model.material("rod_metal", rgba=(0.34, 0.35, 0.38, 1.0))
    diffuser_white = model.material("diffuser_white", rgba=(0.94, 0.95, 0.92, 1.0))
    shade_finish = model.material("shade_finish", rgba=(0.15, 0.16, 0.18, 1.0))

    canopy = model.part("canopy")
    canopy.visual(_build_canopy_shell(), material=canopy_metal, name="canopy_shell")
    canopy.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.046)),
        material=canopy_metal,
        name="rod_socket",
    )
    canopy.visual(
        Cylinder(radius=0.006, length=0.535),
        origin=Origin(xyz=(0.0, 0.0, -0.3255)),
        material=rod_metal,
        name="drop_rod",
    )
    canopy.inertial = Inertial.from_geometry(
        Cylinder(radius=0.070, length=0.640),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, -0.320)),
    )

    diffuser_ring = model.part("diffuser_ring")
    diffuser_ring.visual(
        _build_diffuser_ring_shell(),
        material=diffuser_white,
        name="diffuser_band",
    )
    diffuser_ring.visual(
        Cylinder(radius=0.028, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=canopy_metal,
        name="hub_collar",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0), start=1):
        diffuser_ring.visual(
            Box((0.090, 0.012, 0.006)),
            origin=Origin(xyz=(0.067, 0.0, -0.018), rpy=(0.0, 0.0, angle)),
            material=canopy_metal,
            name=f"spoke_{index}",
        )
    diffuser_ring.visual(
        Box((0.118, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.054)),
        material=canopy_metal,
        name="yoke_bar",
    )
    diffuser_ring.visual(
        Box((0.012, 0.018, 0.048)),
        origin=Origin(xyz=(0.059, 0.0, -0.078)),
        material=canopy_metal,
        name="right_yoke_ear",
    )
    diffuser_ring.visual(
        Box((0.012, 0.018, 0.048)),
        origin=Origin(xyz=(-0.059, 0.0, -0.078)),
        material=canopy_metal,
        name="left_yoke_ear",
    )
    diffuser_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.160, length=0.110),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.0065, length=0.082),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=canopy_metal,
        name="trunnion_rod",
    )
    shade.visual(
        Cylinder(radius=0.009, length=0.012),
        origin=Origin(xyz=(0.047, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=canopy_metal,
        name="right_trunnion_boss",
    )
    shade.visual(
        Cylinder(radius=0.009, length=0.012),
        origin=Origin(xyz=(-0.047, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=canopy_metal,
        name="left_trunnion_boss",
    )
    shade.visual(
        Cylinder(radius=0.046, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, -0.029)),
        material=canopy_metal,
        name="top_cap",
    )
    shade.visual(_build_shade_shell(), material=shade_finish, name="shade_shell")
    shade.inertial = Inertial.from_geometry(
        Cylinder(radius=0.210, length=0.245),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, -0.125)),
    )

    model.articulation(
        "rod_to_diffuser_ring",
        ArticulationType.CONTINUOUS,
        parent=canopy,
        child=diffuser_ring,
        origin=Origin(xyz=(0.0, 0.0, -0.593)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )
    model.articulation(
        "ring_to_shade_tilt",
        ArticulationType.REVOLUTE,
        parent=diffuser_ring,
        child=shade,
        origin=Origin(xyz=(0.0, 0.0, -0.074)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=1.5,
            lower=-math.radians(28.0),
            upper=math.radians(28.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    canopy = object_model.get_part("canopy")
    diffuser_ring = object_model.get_part("diffuser_ring")
    shade = object_model.get_part("shade")
    ring_spin = object_model.get_articulation("rod_to_diffuser_ring")
    shade_tilt = object_model.get_articulation("ring_to_shade_tilt")

    ctx.check(
        "articulations use expected axes",
        ring_spin.axis == (0.0, 0.0, 1.0) and shade_tilt.axis == (1.0, 0.0, 0.0),
        details=f"ring_axis={ring_spin.axis}, shade_axis={shade_tilt.axis}",
    )

    ctx.expect_contact(
        diffuser_ring,
        canopy,
        elem_a="hub_collar",
        elem_b="drop_rod",
        name="diffuser ring hangs from rod base",
    )
    ctx.expect_contact(
        shade,
        diffuser_ring,
        elem_a="right_trunnion_boss",
        elem_b="right_yoke_ear",
        name="right trunnion seats against right yoke ear",
    )
    ctx.expect_contact(
        shade,
        diffuser_ring,
        elem_a="left_trunnion_boss",
        elem_b="left_yoke_ear",
        name="left trunnion seats against left yoke ear",
    )
    ctx.expect_gap(
        diffuser_ring,
        shade,
        axis="z",
        positive_elem="yoke_bar",
        negative_elem="top_cap",
        min_gap=0.015,
        max_gap=0.030,
        name="shade top cap clears the yoke bar",
    )

    rest_ear = _aabb_center(ctx.part_element_world_aabb(diffuser_ring, elem="right_yoke_ear"))
    with ctx.pose({ring_spin: math.pi / 2.0}):
        turned_ear = _aabb_center(
            ctx.part_element_world_aabb(diffuser_ring, elem="right_yoke_ear")
        )
    ctx.check(
        "diffuser ring rotates about the rod axis",
        rest_ear is not None
        and turned_ear is not None
        and rest_ear[0] > 0.045
        and abs(rest_ear[1]) < 0.015
        and turned_ear[1] > 0.045
        and abs(turned_ear[0]) < 0.015,
        details=f"rest_ear={rest_ear}, turned_ear={turned_ear}",
    )

    rest_shell = ctx.part_element_world_aabb(shade, elem="shade_shell")
    rest_shell_center = _aabb_center(rest_shell)
    upper_tilt = shade_tilt.motion_limits.upper if shade_tilt.motion_limits else None
    with ctx.pose({shade_tilt: upper_tilt or 0.0}):
        tilted_shell = ctx.part_element_world_aabb(shade, elem="shade_shell")
        tilted_shell_center = _aabb_center(tilted_shell)
    ctx.check(
        "shade tilts away from vertical at the yoke",
        rest_shell is not None
        and tilted_shell is not None
        and rest_shell_center is not None
        and tilted_shell_center is not None
        and abs(tilted_shell_center[1] - rest_shell_center[1]) > 0.040
        and tilted_shell[1][2] > rest_shell[1][2] + 0.040,
        details=(
            f"rest_center={rest_shell_center}, tilted_center={tilted_shell_center}, "
            f"rest_aabb={rest_shell}, tilted_aabb={tilted_shell}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
