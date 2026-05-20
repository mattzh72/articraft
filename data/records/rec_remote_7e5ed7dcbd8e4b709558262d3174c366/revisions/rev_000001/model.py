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
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _yz_rounded_section(
    x: float,
    *,
    width: float,
    height: float,
    corner_radius: float,
) -> list[tuple[float, float, float]]:
    radius = min(corner_radius, width * 0.48, height * 0.48)
    return [(x, y, z + height * 0.5) for y, z in rounded_rect_profile(width, height, radius)]


def _body_mesh(
    name: str,
    sections: list[tuple[float, float, float, float]],
):
    return mesh_from_geometry(
        section_loft(
            [
                _yz_rounded_section(
                    x,
                    width=width,
                    height=height,
                    corner_radius=corner_radius,
                )
                for x, width, height, corner_radius in sections
            ]
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_presenter_clicker")

    base_black = model.material("base_black", rgba=(0.12, 0.12, 0.13, 1.0))
    top_graphite = model.material("top_graphite", rgba=(0.21, 0.22, 0.24, 1.0))
    guard_black = model.material("guard_black", rgba=(0.08, 0.08, 0.09, 1.0))
    button_green = model.material("button_green", rgba=(0.35, 0.74, 0.28, 1.0))
    accent_grey = model.material("accent_grey", rgba=(0.38, 0.40, 0.43, 1.0))

    lower_base = model.part("lower_base")
    lower_base.visual(
        _body_mesh(
            "lower_base_shell",
            [
                (0.000, 0.036, 0.0106, 0.0044),
                (0.060, 0.036, 0.0110, 0.0044),
                (0.118, 0.034, 0.0102, 0.0040),
            ],
        ),
        material=base_black,
        name="dock_shell",
    )
    lower_base.visual(
        Box((0.076, 0.0032, 0.0046)),
        origin=Origin(xyz=(0.064, -0.0160, 0.0127)),
        material=accent_grey,
        name="left_rail",
    )
    lower_base.visual(
        Box((0.076, 0.0032, 0.0046)),
        origin=Origin(xyz=(0.064, 0.0160, 0.0127)),
        material=accent_grey,
        name="right_rail",
    )
    lower_base.visual(
        Box((0.016, 0.018, 0.0034)),
        origin=Origin(xyz=(0.018, 0.000, 0.0121)),
        material=accent_grey,
        name="rear_stop",
    )
    lower_base.visual(
        Box((0.072, 0.014, 0.0012)),
        origin=Origin(xyz=(0.058, 0.000, 0.0104)),
        material=accent_grey,
        name="track_deck",
    )
    lower_base.inertial = Inertial.from_geometry(
        Box((0.118, 0.036, 0.015)),
        mass=0.14,
        origin=Origin(xyz=(0.059, 0.000, 0.0075)),
    )

    clicker_top = model.part("clicker_top")
    clicker_top.visual(
        _body_mesh(
            "clicker_top_shell",
            [
                (0.000, 0.0248, 0.0124, 0.0048),
                (0.046, 0.0258, 0.0134, 0.0050),
                (0.074, 0.0208, 0.0098, 0.0042),
            ],
        ),
        material=top_graphite,
        name="top_shell",
    )
    clicker_top.visual(
        Cylinder(radius=0.0048, length=0.0020),
        origin=Origin(xyz=(0.051, 0.000, 0.0131)),
        material=button_green,
        name="thumb_button",
    )
    clicker_top.visual(
        Box((0.010, 0.018, 0.0028)),
        origin=Origin(xyz=(0.041, 0.000, 0.0146)),
        material=accent_grey,
        name="guard_pedestal",
    )
    clicker_top.visual(
        Box((0.030, 0.010, 0.0012)),
        origin=Origin(xyz=(0.026, 0.000, 0.0006)),
        material=accent_grey,
        name="glide_pad",
    )
    clicker_top.inertial = Inertial.from_geometry(
        Box((0.074, 0.026, 0.015)),
        mass=0.08,
        origin=Origin(xyz=(0.037, 0.000, 0.0075)),
    )

    button_guard = model.part("button_guard")
    button_guard.visual(
        Cylinder(radius=0.0016, length=0.018),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=guard_black,
        name="guard_barrel",
    )
    button_guard.visual(
        Box((0.016, 0.0024, 0.0024)),
        origin=Origin(xyz=(0.008, -0.0068, -0.0012)),
        material=guard_black,
        name="left_guard_rail",
    )
    button_guard.visual(
        Box((0.016, 0.0024, 0.0024)),
        origin=Origin(xyz=(0.008, 0.0068, -0.0012)),
        material=guard_black,
        name="right_guard_rail",
    )
    button_guard.visual(
        Box((0.0084, 0.0172, 0.0024)),
        origin=Origin(xyz=(0.0120, 0.000, -0.0012)),
        material=guard_black,
        name="guard_bridge",
    )
    button_guard.inertial = Inertial.from_geometry(
        Box((0.020, 0.020, 0.006)),
        mass=0.01,
        origin=Origin(xyz=(0.010, 0.000, -0.001)),
    )

    model.articulation(
        "lower_base_to_clicker_top",
        ArticulationType.PRISMATIC,
        parent=lower_base,
        child=clicker_top,
        origin=Origin(xyz=(0.028, 0.000, 0.0110)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.18,
            lower=0.0,
            upper=0.030,
        ),
    )

    model.articulation(
        "clicker_top_to_button_guard",
        ArticulationType.REVOLUTE,
        parent=clicker_top,
        child=button_guard,
        origin=Origin(xyz=(0.041, 0.000, 0.0176)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.15,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
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

    lower_base = object_model.get_part("lower_base")
    clicker_top = object_model.get_part("clicker_top")
    button_guard = object_model.get_part("button_guard")
    slide = object_model.get_articulation("lower_base_to_clicker_top")
    guard_hinge = object_model.get_articulation("clicker_top_to_button_guard")

    slide_upper = 0.0
    if slide.motion_limits is not None and slide.motion_limits.upper is not None:
        slide_upper = slide.motion_limits.upper

    guard_upper = 0.0
    if guard_hinge.motion_limits is not None and guard_hinge.motion_limits.upper is not None:
        guard_upper = guard_hinge.motion_limits.upper

    with ctx.pose({slide: 0.0, guard_hinge: 0.0}):
        ctx.expect_overlap(
            clicker_top,
            lower_base,
            axes="x",
            min_overlap=0.070,
            elem_a="top_shell",
            elem_b="dock_shell",
            name="docked clicker top covers most of the base length",
        )
        ctx.expect_within(
            clicker_top,
            lower_base,
            axes="y",
            margin=0.001,
            inner_elem="top_shell",
            outer_elem="dock_shell",
            name="docked clicker top stays centered between the base sides",
        )
        ctx.expect_gap(
            clicker_top,
            lower_base,
            axis="z",
            min_gap=0.0,
            max_gap=0.0008,
            positive_elem="glide_pad",
            negative_elem="track_deck",
            name="docked clicker top rides on the lower base track deck",
        )
        ctx.expect_contact(
            clicker_top,
            lower_base,
            elem_a="glide_pad",
            elem_b="track_deck",
            name="slider shoe makes real contact with the track deck at rest",
        )
        ctx.expect_gap(
            button_guard,
            clicker_top,
            axis="z",
            min_gap=0.0010,
            max_gap=0.0040,
            positive_elem="guard_bridge",
            negative_elem="thumb_button",
            name="closed button guard hovers just above the guarded button",
        )
        ctx.expect_overlap(
            button_guard,
            clicker_top,
            axes="xy",
            min_overlap=0.0065,
            elem_a="guard_bridge",
            elem_b="thumb_button",
            name="closed button guard stays aligned over the guarded button",
        )

        rest_top_pos = ctx.part_world_position(clicker_top)
        closed_guard_bridge_aabb = ctx.part_element_world_aabb(button_guard, elem="guard_bridge")

    with ctx.pose({slide: slide_upper, guard_hinge: guard_upper}):
        ctx.expect_overlap(
            clicker_top,
            lower_base,
            axes="x",
            min_overlap=0.040,
            elem_a="top_shell",
            elem_b="dock_shell",
            name="extended clicker top keeps retained insertion on the base",
        )
        ctx.expect_within(
            clicker_top,
            lower_base,
            axes="y",
            margin=0.001,
            inner_elem="top_shell",
            outer_elem="dock_shell",
            name="extended clicker top remains laterally captured by the base",
        )
        ctx.expect_gap(
            button_guard,
            clicker_top,
            axis="z",
            min_gap=0.009,
            positive_elem="guard_bridge",
            negative_elem="thumb_button",
            name="opened guard lifts well clear of the guarded button",
        )

        extended_top_pos = ctx.part_world_position(clicker_top)
        open_guard_bridge_aabb = ctx.part_element_world_aabb(button_guard, elem="guard_bridge")

    ctx.check(
        "clicker top slides forward along the body axis",
        rest_top_pos is not None
        and extended_top_pos is not None
        and extended_top_pos[0] > rest_top_pos[0] + 0.020,
        details=f"rest={rest_top_pos}, extended={extended_top_pos}",
    )
    ctx.check(
        "button guard front rises when opened",
        closed_guard_bridge_aabb is not None
        and open_guard_bridge_aabb is not None
        and open_guard_bridge_aabb[1][2] > closed_guard_bridge_aabb[1][2] + 0.010,
        details=f"closed={closed_guard_bridge_aabb}, open={open_guard_bridge_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
