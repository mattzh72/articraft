from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _merged_mesh(name: str, *geometries):
    merged = geometries[0].copy()
    for geometry in geometries[1:]:
        merged.merge(geometry)
    return mesh_from_geometry(merged, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hand_syringe")

    barrel_clear = model.material("barrel_clear", rgba=(0.88, 0.92, 0.98, 0.45))
    plunger_plastic = model.material("plunger_plastic", rgba=(0.96, 0.96, 0.97, 1.0))
    plunger_seal = model.material("plunger_seal", rgba=(0.22, 0.24, 0.28, 1.0))

    barrel = model.part("barrel")

    barrel_shell = LatheGeometry.from_shell_profiles(
        [
            (0.0086, 0.000),
            (0.0086, 0.094),
            (0.0064, 0.101),
            (0.0038, 0.109),
            (0.0022, 0.118),
        ],
        [
            (0.0072, 0.000),
            (0.0072, 0.091),
            (0.0053, 0.098),
            (0.0011, 0.112),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    rear_ring = TorusGeometry(
        radius=0.0088,
        tube=0.0018,
        radial_segments=16,
        tubular_segments=56,
    ).translate(0.0, 0.0, -0.0016)
    finger_tab_left = BoxGeometry((0.026, 0.0055, 0.0028)).translate(0.0, 0.0108, -0.0020)
    finger_tab_right = BoxGeometry((0.026, 0.0055, 0.0028)).translate(0.0, -0.0108, -0.0020)

    barrel.visual(
        _merged_mesh("barrel_shell", barrel_shell),
        material=barrel_clear,
        name="barrel_shell",
    )
    barrel.visual(
        _merged_mesh("rear_ring", rear_ring),
        material=barrel_clear,
        name="rear_ring",
    )
    barrel.visual(
        _merged_mesh("finger_tab_left", finger_tab_left),
        material=barrel_clear,
        name="finger_tab_left",
    )
    barrel.visual(
        _merged_mesh("finger_tab_right", finger_tab_right),
        material=barrel_clear,
        name="finger_tab_right",
    )
    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.013, length=0.122),
        mass=0.045,
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0018, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=plunger_plastic,
        name="plunger_rod",
    )
    plunger.visual(
        Cylinder(radius=0.0045, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.064)),
        material=plunger_plastic,
        name="rod_boss",
    )
    plunger.visual(
        Cylinder(radius=0.0135, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.073)),
        material=plunger_plastic,
        name="thumb_pad",
    )
    plunger.visual(
        Cylinder(radius=0.0072, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=plunger_seal,
        name="plunger_head",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((0.027, 0.027, 0.098)),
        mass=0.018,
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
    )

    model.articulation(
        "barrel_to_plunger",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.20,
            lower=0.0,
            upper=0.060,
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

    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("barrel_to_plunger")

    ctx.expect_within(
        plunger,
        barrel,
        axes="xy",
        inner_elem="plunger_head",
        outer_elem="barrel_shell",
        margin=0.002,
        name="plunger head stays centered inside the barrel at rest",
    )

    rest_position = ctx.part_world_position(plunger)
    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_within(
            plunger,
            barrel,
            axes="xy",
            inner_elem="plunger_head",
            outer_elem="barrel_shell",
            margin=0.002,
            name="plunger head stays centered at full depression",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="z",
            elem_a="plunger_head",
            elem_b="barrel_shell",
            min_overlap=0.008,
            name="plunger head remains inserted in the barrel at full depression",
        )
        ctx.expect_gap(
            barrel,
            plunger,
            axis="z",
            positive_elem="rear_ring",
            negative_elem="thumb_pad",
            min_gap=0.001,
            max_gap=0.020,
            name="thumb pad stays behind the rear finger flange when depressed",
        )
        depressed_position = ctx.part_world_position(plunger)

    ctx.check(
        "plunger translates toward the nozzle",
        rest_position is not None
        and depressed_position is not None
        and depressed_position[2] > rest_position[2] + 0.05,
        details=f"rest={rest_position}, depressed={depressed_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
