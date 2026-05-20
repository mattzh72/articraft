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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _outer_barrel_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.0405, -0.003),
            (0.0405, 0.000),
            (0.0392, 0.004),
            (0.0392, 0.014),
            (0.0386, 0.016),
            (0.0386, 0.034),
            (0.0398, 0.036),
            (0.0398, 0.053),
            (0.0388, 0.058),
            (0.0375, 0.064),
        ],
        [
            (0.0348, -0.002),
            (0.0348, 0.018),
            (0.0346, 0.040),
            (0.0344, 0.064),
        ],
        segments=88,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _aperture_ring_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.0436, -0.008),
            (0.0443, -0.0065),
            (0.0438, -0.0050),
            (0.0445, -0.0035),
            (0.0438, -0.0020),
            (0.0445, -0.0005),
            (0.0438, 0.0010),
            (0.0445, 0.0025),
            (0.0438, 0.0040),
            (0.0443, 0.0060),
            (0.0436, 0.0080),
        ],
        [
            (0.0397, -0.0085),
            (0.0397, 0.0085),
        ],
        segments=88,
        start_cap="flat",
        end_cap="flat",
    )


def _focus_barrel_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.0328, -0.060),
            (0.0328, -0.012),
            (0.0330, 0.006),
            (0.0340, 0.012),
            (0.0350, 0.028),
            (0.0358, 0.040),
            (0.0362, 0.045),
        ],
        [
            (0.0265, -0.058),
            (0.0265, 0.010),
            (0.0268, 0.028),
            (0.0272, 0.045),
        ],
        segments=88,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="macro_prime_lens")

    anodized_black = model.material("anodized_black", rgba=(0.08, 0.08, 0.09, 1.0))
    satin_black = model.material("satin_black", rgba=(0.13, 0.13, 0.14, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.23, 0.25, 1.0))
    paint_white = model.material("paint_white", rgba=(0.92, 0.92, 0.92, 1.0))
    coating_glass = model.material("coating_glass", rgba=(0.26, 0.38, 0.48, 0.42))

    outer_barrel = model.part("outer_barrel")
    outer_barrel.visual(
        _save_mesh("outer_barrel_shell", _outer_barrel_mesh()),
        material=anodized_black,
        name="outer_sleeve",
    )
    outer_barrel.visual(
        Cylinder(radius=0.0415, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=dark_metal,
        name="mount_flange",
    )
    outer_barrel.visual(
        Box((0.0018, 0.0022, 0.0075)),
        origin=Origin(xyz=(0.0, 0.0399, 0.035)),
        material=paint_white,
        name="aperture_index_mark",
    )

    aperture_ring = model.part("aperture_ring")
    aperture_ring.visual(
        _save_mesh("aperture_ring_band", _aperture_ring_mesh()),
        material=satin_black,
        name="aperture_band",
    )
    aperture_ring.visual(
        Box((0.0012, 0.0050, 0.0018)),
        origin=Origin(xyz=(0.0, 0.0440, 0.0)),
        material=paint_white,
        name="aperture_pointer",
    )

    focus_barrel = model.part("focus_barrel")
    focus_barrel.visual(
        _save_mesh("focus_barrel_shell", _focus_barrel_mesh()),
        material=anodized_black,
        name="focus_tube",
    )
    focus_barrel.visual(
        Cylinder(radius=0.0274, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material=coating_glass,
        name="front_element",
    )

    model.articulation(
        "aperture_rotation",
        ArticulationType.CONTINUOUS,
        parent=outer_barrel,
        child=aperture_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=8.0),
    )
    model.articulation(
        "focus_extension",
        ArticulationType.PRISMATIC,
        parent=outer_barrel,
        child=focus_barrel,
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.08,
            lower=-0.015,
            upper=0.030,
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

    outer_barrel = object_model.get_part("outer_barrel")
    aperture_ring = object_model.get_part("aperture_ring")
    focus_barrel = object_model.get_part("focus_barrel")
    aperture_rotation = object_model.get_articulation("aperture_rotation")
    focus_extension = object_model.get_articulation("focus_extension")

    ctx.check(
        "aperture ring rotates about optical axis",
        aperture_rotation.articulation_type == ArticulationType.CONTINUOUS
        and tuple(aperture_rotation.axis) == (0.0, 0.0, 1.0),
        details=f"type={aperture_rotation.articulation_type}, axis={aperture_rotation.axis}",
    )
    ctx.check(
        "focus barrel slides along optical axis",
        focus_extension.articulation_type == ArticulationType.PRISMATIC
        and tuple(focus_extension.axis) == (0.0, 0.0, 1.0),
        details=f"type={focus_extension.articulation_type}, axis={focus_extension.axis}",
    )

    ctx.expect_within(
        focus_barrel,
        outer_barrel,
        axes="xy",
        inner_elem="focus_tube",
        outer_elem="outer_sleeve",
        margin=0.002,
        name="focus barrel stays centered inside outer barrel",
    )
    ctx.expect_overlap(
        focus_barrel,
        outer_barrel,
        axes="z",
        elem_a="focus_tube",
        elem_b="outer_sleeve",
        min_overlap=0.055,
        name="focus barrel remains inserted at rest",
    )
    ctx.expect_overlap(
        aperture_ring,
        outer_barrel,
        axes="z",
        elem_a="aperture_band",
        elem_b="outer_sleeve",
        min_overlap=0.014,
        name="aperture ring spans a real barrel section",
    )

    rest_focus_pos = ctx.part_world_position(focus_barrel)
    with ctx.pose({focus_extension: 0.030}):
        ctx.expect_within(
            focus_barrel,
            outer_barrel,
            axes="xy",
            inner_elem="focus_tube",
            outer_elem="outer_sleeve",
            margin=0.002,
            name="extended focus barrel stays guided by the outer sleeve",
        )
        ctx.expect_overlap(
            focus_barrel,
            outer_barrel,
            axes="z",
            elem_a="focus_tube",
            elem_b="outer_sleeve",
            min_overlap=0.034,
            name="extended focus barrel keeps retained insertion",
        )
        extended_focus_pos = ctx.part_world_position(focus_barrel)

    with ctx.pose({focus_extension: -0.015}):
        retracted_focus_pos = ctx.part_world_position(focus_barrel)

    ctx.check(
        "focus barrel extends forward for close focusing",
        rest_focus_pos is not None
        and extended_focus_pos is not None
        and extended_focus_pos[2] > rest_focus_pos[2] + 0.025,
        details=f"rest={rest_focus_pos}, extended={extended_focus_pos}",
    )
    ctx.check(
        "focus barrel retracts back into the housing",
        rest_focus_pos is not None
        and retracted_focus_pos is not None
        and retracted_focus_pos[2] < rest_focus_pos[2] - 0.010,
        details=f"rest={rest_focus_pos}, retracted={retracted_focus_pos}",
    )

    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))

    rest_mark = aabb_center(ctx.part_element_world_aabb(aperture_ring, elem="aperture_pointer"))
    with ctx.pose({aperture_rotation: math.pi / 2.0}):
        turned_mark = aabb_center(ctx.part_element_world_aabb(aperture_ring, elem="aperture_pointer"))

    ctx.check(
        "aperture pointer orbits when the ring turns",
        rest_mark is not None
        and turned_mark is not None
        and math.hypot(turned_mark[0] - rest_mark[0], turned_mark[1] - rest_mark[1]) > 0.050
        and abs(turned_mark[2] - rest_mark[2]) < 0.002,
        details=f"rest={rest_mark}, turned={turned_mark}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
