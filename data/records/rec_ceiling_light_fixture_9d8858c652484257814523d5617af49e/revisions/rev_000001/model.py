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


def _build_canopy_shell() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.011, 0.000),
            (0.032, -0.002),
            (0.054, -0.012),
            (0.061, -0.030),
            (0.058, -0.044),
        ],
        [
            (0.011, 0.000),
            (0.028, -0.004),
            (0.049, -0.013),
            (0.055, -0.030),
            (0.058, -0.044),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _build_cord_grip() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.0110, 0.000),
            (0.0126, -0.006),
            (0.0108, -0.014),
            (0.0115, -0.032),
            (0.0120, -0.038),
            (0.0090, -0.044),
        ],
        [
            (0.0055, 0.000),
            (0.0062, -0.004),
            (0.0062, -0.032),
            (0.0052, -0.044),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )


def _build_shade_shell() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.018, 0.000),
            (0.040, -0.008),
            (0.112, -0.038),
            (0.182, -0.102),
            (0.212, -0.176),
            (0.214, -0.222),
        ],
        [
            (0.000, -0.002),
            (0.030, -0.010),
            (0.102, -0.040),
            (0.172, -0.102),
            (0.204, -0.176),
            (0.214, -0.222),
        ],
        segments=88,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dome_pendant_light")

    painted_white = model.material("painted_white", rgba=(0.93, 0.93, 0.91, 1.0))
    satin_black = model.material("satin_black", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.18, 0.18, 0.20, 1.0))

    canopy = model.part("canopy")
    canopy.visual(
        mesh_from_geometry(_build_canopy_shell(), "ceiling_canopy"),
        material=painted_white,
        name="canopy_shell",
    )
    canopy.visual(
        mesh_from_geometry(_build_cord_grip(), "cord_grip"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_graphite,
        name="cord_grip",
    )
    canopy.visual(
        Box((0.0022, 0.0020, 0.020)),
        origin=Origin(xyz=(0.0060, 0.0, -0.034)),
        material=satin_black,
        name="grip_pad",
    )
    canopy.inertial = Inertial.from_geometry(
        Cylinder(radius=0.062, length=0.048),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
    )

    hanging_cord = model.part("hanging_cord")
    hanging_cord.visual(
        Cylinder(radius=0.0042, length=0.435),
        origin=Origin(xyz=(0.0, 0.0, -0.1375)),
        material=satin_black,
        name="cord_line",
    )
    hanging_cord.visual(
        Box((0.0032, 0.0020, 0.090)),
        origin=Origin(xyz=(0.0033, 0.0, 0.045)),
        material=satin_black,
        name="slider_key",
    )
    hanging_cord.visual(
        Cylinder(radius=0.010, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.336)),
        material=dark_graphite,
        name="coupling_body",
    )
    hanging_cord.visual(
        Box((0.024, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.346)),
        material=dark_graphite,
        name="yoke_bridge",
    )
    hanging_cord.visual(
        Box((0.004, 0.012, 0.024)),
        origin=Origin(xyz=(-0.007, 0.0, -0.362)),
        material=dark_graphite,
        name="left_yoke_cheek",
    )
    hanging_cord.visual(
        Box((0.004, 0.012, 0.024)),
        origin=Origin(xyz=(0.007, 0.0, -0.362)),
        material=dark_graphite,
        name="right_yoke_cheek",
    )
    hanging_cord.inertial = Inertial.from_geometry(
        Box((0.028, 0.028, 0.445)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, -0.145)),
    )

    shade = model.part("shade")
    shade.visual(
        Box((0.010, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=dark_graphite,
        name="trunnion_block",
    )
    shade.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=dark_graphite,
        name="top_collar",
    )
    shade.visual(
        mesh_from_geometry(_build_shade_shell(), "dome_shade_shell"),
        origin=Origin(xyz=(0.0, 0.0, -0.028)),
        material=painted_white,
        name="shade_shell",
    )
    shade.inertial = Inertial.from_geometry(
        Cylinder(radius=0.214, length=0.250),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, -0.140)),
    )

    grip_slide = model.articulation(
        "canopy_to_hanging_cord",
        ArticulationType.PRISMATIC,
        parent=canopy,
        child=hanging_cord,
        origin=Origin(xyz=(0.0, 0.0, -0.048)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.08,
            lower=0.0,
            upper=0.06,
        ),
    )
    shade_tilt = model.articulation(
        "cord_to_shade_tilt",
        ArticulationType.REVOLUTE,
        parent=hanging_cord,
        child=shade,
        origin=Origin(xyz=(0.0, 0.0, -0.374)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=math.radians(-18.0),
            upper=math.radians(34.0),
        ),
    )

    model.meta["prompt_articulations"] = {
        "grip_slide": grip_slide.name,
        "shade_tilt": shade_tilt.name,
    }
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

    canopy = object_model.get_part("canopy")
    hanging_cord = object_model.get_part("hanging_cord")
    shade = object_model.get_part("shade")
    grip_slide = object_model.get_articulation("canopy_to_hanging_cord")
    shade_tilt = object_model.get_articulation("cord_to_shade_tilt")

    ctx.expect_contact(
        hanging_cord,
        canopy,
        elem_a="slider_key",
        elem_b="grip_pad",
        name="cord remains seated in the canopy grip at rest",
    )
    ctx.expect_within(
        hanging_cord,
        canopy,
        axes="xy",
        inner_elem="cord_line",
        outer_elem="cord_grip",
        margin=0.001,
        name="cord runs centered through the canopy grip",
    )
    ctx.expect_contact(
        shade,
        hanging_cord,
        elem_a="trunnion_block",
        name="shade trunnion is supported by the cord yoke",
    )

    rest_shade_pos = ctx.part_world_position(shade)
    with ctx.pose({grip_slide: grip_slide.motion_limits.upper}):
        ctx.expect_contact(
            hanging_cord,
            canopy,
            elem_a="slider_key",
            elem_b="grip_pad",
            name="cord remains captured in the grip at max extension",
        )
        ctx.expect_overlap(
            hanging_cord,
            canopy,
            axes="z",
            elem_a="cord_line",
            elem_b="cord_grip",
            min_overlap=0.012,
            name="cord retains insertion through the grip sleeve",
        )
        extended_shade_pos = ctx.part_world_position(shade)

    ctx.check(
        "prismatic grip joint lowers the shade",
        rest_shade_pos is not None
        and extended_shade_pos is not None
        and extended_shade_pos[2] < rest_shade_pos[2] - 0.04,
        details=f"rest={rest_shade_pos}, extended={extended_shade_pos}",
    )

    rest_aabb = ctx.part_world_aabb(shade)
    with ctx.pose({shade_tilt: shade_tilt.motion_limits.upper}):
        tilted_aabb = ctx.part_world_aabb(shade)

    ctx.check(
        "shade tilt raises one side of the dome",
        rest_aabb is not None
        and tilted_aabb is not None
        and tilted_aabb[1][2] > rest_aabb[1][2] + 0.03,
        details=f"rest={rest_aabb}, tilted={tilted_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
