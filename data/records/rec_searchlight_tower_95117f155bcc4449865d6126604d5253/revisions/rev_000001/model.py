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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="searchlight_tower")

    tower_gray = model.material("tower_gray", rgba=(0.42, 0.45, 0.48, 1.0))
    machinery_gray = model.material("machinery_gray", rgba=(0.32, 0.34, 0.37, 1.0))
    lamp_paint = model.material("lamp_paint", rgba=(0.50, 0.53, 0.50, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.74, 0.84, 0.92, 0.70))

    tower = model.part("tower")
    tower.visual(
        Box((1.50, 1.50, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=tower_gray,
        name="base_plinth",
    )
    tower.visual(
        Box((0.88, 0.88, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        material=machinery_gray,
        name="pedestal_block",
    )
    tower.visual(
        Box((0.40, 0.40, 1.60)),
        origin=Origin(xyz=(0.0, 0.0, 1.26)),
        material=tower_gray,
        name="mast_column",
    )
    tower.visual(
        Cylinder(radius=0.24, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 2.12)),
        material=machinery_gray,
        name="mast_cap",
    )
    tower.inertial = Inertial.from_geometry(
        Box((1.50, 1.50, 2.18)),
        mass=3200.0,
        origin=Origin(xyz=(0.0, 0.0, 1.09)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.22, length=0.08),
        origin=Origin(),
        material=machinery_gray,
        name="turntable",
    )
    yoke.visual(
        Box((0.18, 0.30, 0.46)),
        origin=Origin(xyz=(-0.06, 0.0, 0.27)),
        material=tower_gray,
        name="support_pedestal",
    )
    yoke.visual(
        Box((0.48, 0.58, 0.08)),
        origin=Origin(xyz=(0.15, 0.0, 0.54)),
        material=tower_gray,
        name="crosshead",
    )
    yoke.visual(
        Box((0.18, 0.08, 0.44)),
        origin=Origin(xyz=(0.30, 0.24, 0.28)),
        material=tower_gray,
        name="left_arm",
    )
    yoke.visual(
        Box((0.18, 0.08, 0.44)),
        origin=Origin(xyz=(0.30, -0.24, 0.28)),
        material=tower_gray,
        name="right_arm",
    )
    yoke.visual(
        Cylinder(radius=0.045, length=0.04),
        origin=Origin(xyz=(0.30, 0.18, 0.28), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machinery_gray,
        name="left_pivot_barrel",
    )
    yoke.visual(
        Cylinder(radius=0.045, length=0.04),
        origin=Origin(xyz=(0.30, -0.18, 0.28), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machinery_gray,
        name="right_pivot_barrel",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.48, 0.50, 0.58)),
        mass=420.0,
        origin=Origin(xyz=(0.14, 0.0, 0.25)),
    )

    lamp = model.part("lamp")
    lamp.visual(
        Cylinder(radius=0.03, length=0.32),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machinery_gray,
        name="trunnion_shaft",
    )
    lamp.visual(
        Cylinder(radius=0.13, length=0.38),
        origin=Origin(xyz=(0.02, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lamp_paint,
        name="body_shell",
    )
    lamp.visual(
        Cylinder(radius=0.155, length=0.04),
        origin=Origin(xyz=(0.23, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machinery_gray,
        name="front_bezel",
    )
    lamp.visual(
        Cylinder(radius=0.145, length=0.012),
        origin=Origin(xyz=(0.256, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="lens_window",
    )
    lamp.visual(
        Box((0.14, 0.20, 0.20)),
        origin=Origin(xyz=(-0.20, 0.0, 0.0)),
        material=machinery_gray,
        name="rear_housing",
    )
    lamp.inertial = Inertial.from_geometry(
        Box((0.55, 0.30, 0.31)),
        mass=180.0,
        origin=Origin(xyz=(0.02, 0.0, 0.0)),
    )

    model.articulation(
        "tower_to_yoke",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 2.22)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2500.0,
            velocity=0.9,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "yoke_to_lamp",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp,
        origin=Origin(xyz=(0.30, 0.0, 0.28)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1400.0,
            velocity=0.8,
            lower=math.radians(-35.0),
            upper=math.radians(55.0),
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

    tower = object_model.get_part("tower")
    yoke = object_model.get_part("yoke")
    lamp = object_model.get_part("lamp")
    pan = object_model.get_articulation("tower_to_yoke")
    tilt = object_model.get_articulation("yoke_to_lamp")

    ctx.expect_contact(
        yoke,
        tower,
        elem_a="turntable",
        elem_b="mast_cap",
        name="turntable seats on mast cap",
    )
    ctx.expect_gap(
        yoke,
        lamp,
        axis="y",
        positive_elem="left_arm",
        negative_elem="body_shell",
        min_gap=0.045,
        max_gap=0.090,
        name="left yoke arm clears lamp body",
    )
    ctx.expect_gap(
        lamp,
        yoke,
        axis="y",
        positive_elem="body_shell",
        negative_elem="right_arm",
        min_gap=0.045,
        max_gap=0.090,
        name="right yoke arm clears lamp body",
    )

    def elem_center(part_name: str, elem: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem)
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(lower, upper))

    rest_lens = elem_center("lamp", "lens_window")
    with ctx.pose({tilt: math.radians(45.0)}):
        raised_lens = elem_center("lamp", "lens_window")
    ctx.check(
        "tilt raises the front lens",
        rest_lens is not None
        and raised_lens is not None
        and raised_lens[2] > rest_lens[2] + 0.12,
        details=f"rest={rest_lens}, raised={raised_lens}",
    )

    rest_pan_lens = elem_center("lamp", "lens_window")
    with ctx.pose({pan: math.pi / 2.0}):
        swung_lens = elem_center("lamp", "lens_window")
    ctx.check(
        "pan swings the lamp around the mast",
        rest_pan_lens is not None
        and swung_lens is not None
        and swung_lens[1] > rest_pan_lens[1] + 0.40
        and abs(swung_lens[0]) < 0.15,
        details=f"rest={rest_pan_lens}, swung={swung_lens}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
