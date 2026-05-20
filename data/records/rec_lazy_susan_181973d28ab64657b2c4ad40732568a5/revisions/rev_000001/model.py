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
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="condiment_lazy_susan")

    tray_wood = model.material("tray_wood", rgba=(0.63, 0.47, 0.29, 1.0))
    base_charcoal = model.material("base_charcoal", rgba=(0.18, 0.18, 0.19, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.76, 0.78, 0.80, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.110, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=base_charcoal,
        name="base_skirt",
    )
    base.visual(
        Cylinder(radius=0.092, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=base_charcoal,
        name="base_body",
    )
    base.visual(
        Cylinder(radius=0.072, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=base_charcoal,
        name="base_cap",
    )
    base.visual(
        Cylinder(radius=0.054, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=brushed_steel,
        name="base_bearing_plate",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.110, length=0.040),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    tray = model.part("tray")
    tray_shell = LatheGeometry.from_shell_profiles(
        [
            (0.040, 0.010),
            (0.070, 0.012),
            (0.115, 0.014),
            (0.145, 0.018),
            (0.149, 0.045),
            (0.145, 0.050),
        ],
        [
            (0.000, 0.016),
            (0.110, 0.016),
            (0.138, 0.018),
            (0.143, 0.046),
        ],
        segments=72,
    )
    tray.visual(
        mesh_from_geometry(tray_shell, "lazy_susan_tray_shell"),
        material=tray_wood,
        name="tray_shell",
    )
    tray.visual(
        Cylinder(radius=0.048, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=tray_wood,
        name="tray_bearing_hub",
    )
    tray.visual(
        Cylinder(radius=0.016, length=0.006),
        origin=Origin(xyz=(-0.098, 0.0, 0.019)),
        material=tray_wood,
        name="left_handle_pad",
    )
    tray.visual(
        Cylinder(radius=0.016, length=0.006),
        origin=Origin(xyz=(0.098, 0.0, 0.019)),
        material=tray_wood,
        name="right_handle_pad",
    )
    tray.inertial = Inertial.from_geometry(
        Cylinder(radius=0.150, length=0.050),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    handle = model.part("handle")
    arch_geom = tube_from_spline_points(
        [
            (-0.098, 0.0, 0.008),
            (-0.098, 0.0, 0.045),
            (-0.060, 0.0, 0.105),
            (0.000, 0.0, 0.145),
            (0.060, 0.0, 0.105),
            (0.098, 0.0, 0.045),
            (0.098, 0.0, 0.008),
        ],
        radius=0.008,
        samples_per_segment=20,
        radial_segments=18,
        cap_ends=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    handle.visual(
        mesh_from_geometry(arch_geom, "lazy_susan_handle_arch"),
        material=brushed_steel,
        name="handle_arch",
    )
    handle.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(-0.098, 0.0, 0.004)),
        material=brushed_steel,
        name="left_handle_foot",
    )
    handle.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.098, 0.0, 0.004)),
        material=brushed_steel,
        name="right_handle_foot",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.220, 0.030, 0.155)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
    )

    model.articulation(
        "base_to_tray",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=4.0),
    )
    model.articulation(
        "tray_to_handle",
        ArticulationType.FIXED,
        parent=tray,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
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

    base = object_model.get_part("base")
    tray = object_model.get_part("tray")
    handle = object_model.get_part("handle")
    spin = object_model.get_articulation("base_to_tray")

    ctx.check(
        "tray uses continuous vertical rotation",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(spin.axis) == (0.0, 0.0, 1.0)
        and spin.motion_limits is not None
        and spin.motion_limits.lower is None
        and spin.motion_limits.upper is None,
        details=(
            f"type={spin.articulation_type}, axis={spin.axis}, "
            f"limits={spin.motion_limits}"
        ),
    )
    ctx.expect_contact(
        tray,
        base,
        elem_a="tray_bearing_hub",
        elem_b="base_bearing_plate",
        name="tray hub sits on base bearing plate",
    )
    ctx.expect_contact(
        handle,
        tray,
        elem_a="left_handle_foot",
        elem_b="left_handle_pad",
        name="left handle foot mounts to tray pad",
    )
    ctx.expect_contact(
        handle,
        tray,
        elem_a="right_handle_foot",
        elem_b="right_handle_pad",
        name="right handle foot mounts to tray pad",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    rest_center = _aabb_center(ctx.part_element_world_aabb(handle, elem="left_handle_foot"))
    with ctx.pose({spin: math.pi / 2.0}):
        turned_center = _aabb_center(ctx.part_element_world_aabb(handle, elem="left_handle_foot"))
        ctx.expect_contact(
            tray,
            base,
            elem_a="tray_bearing_hub",
            elem_b="base_bearing_plate",
            name="tray stays seated on base bearing after quarter turn",
        )

    ctx.check(
        "quarter turn moves handle foot around vertical axis",
        rest_center is not None
        and turned_center is not None
        and rest_center[0] < -0.07
        and abs(rest_center[1]) < 0.02
        and turned_center[1] < -0.07
        and abs(turned_center[0]) < 0.02
        and abs(rest_center[2] - turned_center[2]) < 0.002,
        details=f"rest_center={rest_center}, turned_center={turned_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
