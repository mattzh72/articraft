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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lazy_susan")

    board_wood = model.material("board_wood", rgba=(0.76, 0.60, 0.39, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.66, 0.68, 0.71, 1.0))
    dark_base = model.material("dark_base", rgba=(0.18, 0.18, 0.19, 1.0))

    top_board_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.34, 0.34, 0.018, corner_segments=8),
            0.018,
            center=True,
            cap=True,
        ),
        "lazy_susan_top_board",
    )

    turntable_base = model.part("turntable_base")
    turntable_base.visual(
        Cylinder(radius=0.115, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_base,
        name="base_pad",
    )
    turntable_base.visual(
        Cylinder(radius=0.108, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=bearing_steel,
        name="bearing_shoulder",
    )
    turntable_base.visual(
        Cylinder(radius=0.100, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=dark_base,
        name="bearing_drum",
    )
    turntable_base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.115, length=0.022),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
    )

    rotating_top = model.part("rotating_top")
    rotating_top.visual(
        Cylinder(radius=0.092, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=bearing_steel,
        name="mounting_plate",
    )
    rotating_top.visual(
        top_board_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=board_wood,
        name="top_board",
    )
    rotating_top.inertial = Inertial.from_geometry(
        Box((0.34, 0.34, 0.024)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    model.articulation(
        "base_to_top",
        ArticulationType.CONTINUOUS,
        parent=turntable_base,
        child=rotating_top,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
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

    turntable_base = object_model.get_part("turntable_base")
    rotating_top = object_model.get_part("rotating_top")
    base_to_top = object_model.get_articulation("base_to_top")

    ctx.check(
        "continuous center rotation is configured",
        base_to_top.joint_type == ArticulationType.CONTINUOUS
        and base_to_top.axis == (0.0, 0.0, 1.0)
        and base_to_top.motion_limits is not None
        and base_to_top.motion_limits.lower is None
        and base_to_top.motion_limits.upper is None,
        details=(
            f"type={base_to_top.joint_type}, axis={base_to_top.axis}, "
            f"limits={base_to_top.motion_limits}"
        ),
    )

    ctx.expect_contact(
        rotating_top,
        turntable_base,
        elem_a="mounting_plate",
        elem_b="bearing_drum",
        contact_tol=1e-6,
        name="mounting plate seats on the bearing drum",
    )
    ctx.expect_gap(
        rotating_top,
        turntable_base,
        axis="z",
        positive_elem="top_board",
        negative_elem="bearing_drum",
        min_gap=0.005,
        max_gap=0.007,
        name="square board clears the round bearing hardware",
    )
    ctx.expect_overlap(
        rotating_top,
        turntable_base,
        axes="xy",
        elem_a="top_board",
        elem_b="base_pad",
        min_overlap=0.22,
        name="square top stays centered over the hidden round base",
    )

    rest_board_aabb = ctx.part_element_world_aabb(rotating_top, elem="top_board")
    rest_pos = ctx.part_world_position(rotating_top)
    with ctx.pose({base_to_top: math.pi / 4.0}):
        ctx.expect_contact(
            rotating_top,
            turntable_base,
            elem_a="mounting_plate",
            elem_b="bearing_drum",
            contact_tol=1e-6,
            name="bearing support remains seated after rotation",
        )
        turned_board_aabb = ctx.part_element_world_aabb(rotating_top, elem="top_board")
        turned_pos = ctx.part_world_position(rotating_top)

    def _span(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        return aabb[1][axis_index] - aabb[0][axis_index]

    def _center(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        return (aabb[0][axis_index] + aabb[1][axis_index]) * 0.5

    rest_width_x = _span(rest_board_aabb, 0)
    turned_width_x = _span(turned_board_aabb, 0)
    rest_center_x = _center(rest_board_aabb, 0)
    turned_center_x = _center(turned_board_aabb, 0)
    rest_center_y = _center(rest_board_aabb, 1)
    turned_center_y = _center(turned_board_aabb, 1)
    centered_in_place = (
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) < 1e-6
        and abs(rest_pos[1] - turned_pos[1]) < 1e-6
        and rest_center_x is not None
        and turned_center_x is not None
        and rest_center_y is not None
        and turned_center_y is not None
        and abs(rest_center_x - turned_center_x) < 1e-6
        and abs(rest_center_y - turned_center_y) < 1e-6
    )
    ctx.check(
        "square board spins in place",
        centered_in_place
        and rest_width_x is not None
        and turned_width_x is not None
        and turned_width_x > rest_width_x + 0.10,
        details=(
            f"rest_width_x={rest_width_x}, turned_width_x={turned_width_x}, "
            f"rest_pos={rest_pos}, turned_pos={turned_pos}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
