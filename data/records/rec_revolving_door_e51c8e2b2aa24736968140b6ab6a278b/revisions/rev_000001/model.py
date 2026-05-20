from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_four_wing_revolving_door")

    stainless = model.material("brushed_stainless", rgba=(0.68, 0.69, 0.67, 1.0))
    dark_steel = model.material("dark_bearing_shadow", rgba=(0.16, 0.17, 0.17, 1.0))
    glass = model.material("slightly_blue_glass", rgba=(0.58, 0.82, 0.95, 0.36))
    rubber = model.material("black_floor_seal", rgba=(0.025, 0.025, 0.023, 1.0))

    fixed_frame = model.part("fixed_frame")
    # Low circular threshold plate and a heavy overhead stainless canopy, tied
    # together by perimeter posts so the stationary frame reads as one assembly.
    fixed_frame.visual(
        Cylinder(radius=1.30, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_steel,
        name="round_threshold",
    )
    fixed_frame.visual(
        Cylinder(radius=1.30, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 2.460)),
        material=stainless,
        name="ceiling_canopy",
    )
    for i, yaw in enumerate((math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)):
        x = 1.23 * math.cos(yaw)
        y = 1.23 * math.sin(yaw)
        fixed_frame.visual(
            Cylinder(radius=0.035, length=2.410),
            origin=Origin(xyz=(x, y, 1.235)),
            material=stainless,
            name=f"perimeter_post_{i}",
        )

    carousel = model.part("carousel")
    carousel.visual(
        Cylinder(radius=0.075, length=2.360),
        origin=Origin(xyz=(0.0, 0.0, 1.220)),
        material=stainless,
        name="round_post",
    )
    carousel.visual(
        Cylinder(radius=0.165, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=stainless,
        name="lower_rotating_hub",
    )
    carousel.visual(
        Cylinder(radius=0.140, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 2.335)),
        material=stainless,
        name="upper_rotating_hub",
    )

    def add_wing(index: int, yaw: float) -> None:
        """Add one radial rectangular stainless/glass door wing."""
        c = math.cos(yaw)
        s = math.sin(yaw)

        def xy(local_x: float, local_y: float) -> tuple[float, float, float]:
            return (local_x * c - local_y * s, local_x * s + local_y * c, 0.0)

        def box_visual(name: str, size: tuple[float, float, float], local_center: tuple[float, float, float], material: Material) -> None:
            x, y, _ = xy(local_center[0], local_center[1])
            carousel.visual(
                Box(size),
                origin=Origin(xyz=(x, y, local_center[2]), rpy=(0.0, 0.0, yaw)),
                material=material,
                name=f"wing_{index}_{name}",
            )

        # Heavy rectangular stainless perimeter framing.
        box_visual("inner_stile", (0.090, 0.080, 2.060), (0.112, 0.0, 1.155), stainless)
        box_visual("outer_stile", (0.080, 0.080, 2.060), (1.110, 0.0, 1.155), stainless)
        box_visual("bottom_rail", (1.070, 0.075, 0.085), (0.610, 0.0, 0.165), stainless)
        box_visual("top_rail", (1.070, 0.075, 0.085), (0.610, 0.0, 2.145), stainless)

        # Slightly inset translucent glass, with hidden overlap under the rails
        # so it is visibly captured by the metal frame rather than floating.
        box_visual("glass", (0.930, 0.014, 1.895), (0.610, 0.0, 1.155), glass)

        # Horizontal push bars on both faces of each wing, with short stand-off
        # brackets tying the tubes back into the panel/frame.
        for side, local_y in (("face_a", 0.078), ("face_b", -0.078)):
            x, y, _ = xy(0.610, local_y)
            carousel.visual(
                Cylinder(radius=0.018, length=0.760),
                origin=Origin(xyz=(x, y, 1.050), rpy=(0.0, math.pi / 2.0, yaw)),
                material=stainless,
                name=f"wing_{index}_push_bar_{side}",
            )
            for bracket_x in (0.345, 0.875):
                bx, by, _ = xy(bracket_x, local_y * 0.42)
                box_visual(
                    f"bar_bracket_{side}_{bracket_x:.3f}",
                    (0.050, 0.064, 0.115),
                    (bracket_x, local_y * 0.42, 1.050),
                    stainless,
                )

        # Slim black sweep seals at the vertical outer edge and bottom of the
        # panel, common on commercial revolving-door wings.
        box_visual("outer_seal", (0.025, 0.026, 2.000), (1.162, 0.0, 1.155), rubber)
        box_visual("bottom_sweep", (0.965, 0.025, 0.030), (0.635, 0.0, 0.095), rubber)

    for wing_index in range(4):
        add_wing(wing_index, wing_index * math.pi / 2.0)

    model.articulation(
        "rotation_axis",
        ArticulationType.CONTINUOUS,
        parent=fixed_frame,
        child=carousel,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=1.2),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    fixed_frame = object_model.get_part("fixed_frame")
    carousel = object_model.get_part("carousel")
    rotation_axis = object_model.get_articulation("rotation_axis")

    ctx.expect_contact(
        carousel,
        fixed_frame,
        elem_a="lower_rotating_hub",
        elem_b="round_threshold",
        contact_tol=0.002,
        name="rotating hub bears on threshold",
    )
    ctx.expect_gap(
        fixed_frame,
        carousel,
        axis="z",
        positive_elem="ceiling_canopy",
        negative_elem="upper_rotating_hub",
        min_gap=0.015,
        max_gap=0.050,
        name="upper hub clears overhead canopy",
    )
    ctx.expect_overlap(
        carousel,
        fixed_frame,
        axes="x",
        elem_a="wing_0_glass",
        elem_b="round_threshold",
        min_overlap=0.70,
        name="wing radial span sits over threshold",
    )

    def elem_center_xy(elem_name: str) -> tuple[float, float] | None:
        aabb = ctx.part_element_world_aabb(carousel, elem=elem_name)
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5)

    with ctx.pose({rotation_axis: 0.0}):
        rest_center = elem_center_xy("wing_0_glass")
    with ctx.pose({rotation_axis: math.pi / 2.0}):
        turned_center = elem_center_xy("wing_0_glass")

    ctx.check(
        "wing assembly rotates continuously about post axis",
        rest_center is not None
        and turned_center is not None
        and rest_center[0] > 0.55
        and abs(rest_center[1]) < 0.08
        and turned_center[1] > 0.55
        and abs(turned_center[0]) < 0.08,
        details=f"rest={rest_center}, turned={turned_center}",
    )

    return ctx.report()


object_model = build_object_model()
