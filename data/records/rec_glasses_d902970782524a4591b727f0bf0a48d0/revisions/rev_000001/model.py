from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    ExtrudeWithHolesGeometry,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
)


FRAME_WIDTH = 0.148
FRAME_HEIGHT = 0.058
FRAME_THICKNESS = 0.0065
HINGE_X = 0.074
HINGE_Y = -0.0037
HINGE_Z = 0.008


def _smooth_closed(points: list[tuple[float, float]], samples: int = 10) -> list[tuple[float, float]]:
    return sample_catmull_rom_spline_2d(
        points,
        samples_per_segment=samples,
        closed=True,
    )


def _mirror_x(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return [(-x, z) for x, z in reversed(points)]


def _front_outer_profile() -> list[tuple[float, float]]:
    return _smooth_closed(
        [
            (-0.071, 0.026),
            (-0.074, 0.010),
            (-0.072, -0.010),
            (-0.066, -0.022),
            (-0.047, -0.027),
            (-0.019, -0.023),
            (-0.007, -0.013),
            (0.000, -0.010),
            (0.007, -0.013),
            (0.019, -0.023),
            (0.047, -0.027),
            (0.066, -0.022),
            (0.072, -0.010),
            (0.074, 0.010),
            (0.071, 0.026),
            (0.052, 0.031),
            (0.024, 0.029),
            (0.010, 0.023),
            (0.000, 0.021),
            (-0.010, 0.023),
            (-0.024, 0.029),
            (-0.052, 0.031),
        ],
        samples=8,
    )


def _left_lens_profile() -> list[tuple[float, float]]:
    return _smooth_closed(
        [
            (-0.057, 0.012),
            (-0.058, -0.002),
            (-0.053, -0.015),
            (-0.037, -0.019),
            (-0.021, -0.017),
            (-0.015, -0.006),
            (-0.016, 0.009),
            (-0.024, 0.017),
            (-0.040, 0.018),
            (-0.052, 0.016),
        ],
        samples=8,
    )


def _build_front_shell_mesh():
    front_shell = ExtrudeWithHolesGeometry(
        _front_outer_profile(),
        [_left_lens_profile(), _mirror_x(_left_lens_profile())],
        FRAME_THICKNESS,
        center=True,
        cap=True,
        closed=True,
    )
    front_shell.rotate_x(pi / 2.0)
    return mesh_from_geometry(front_shell, "wayfarer_front_shell")


def _temple_side_profile() -> list[tuple[float, float]]:
    return _smooth_closed(
        [
            (-0.0015, 0.0088),
            (-0.016, 0.0095),
            (-0.060, 0.0092),
            (-0.102, 0.0078),
            (-0.136, 0.0060),
            (-0.150, 0.0048),
            (-0.148, -0.0040),
            (-0.132, -0.0058),
            (-0.098, -0.0075),
            (-0.056, -0.0086),
            (-0.014, -0.0089),
            (-0.0015, -0.0081),
        ],
        samples=8,
    )


def _build_temple_main_mesh(name: str):
    temple = ExtrudeGeometry(
        _temple_side_profile(),
        0.0054,
        center=True,
        cap=True,
        closed=True,
    )
    temple.rotate_y(pi / 2.0)
    temple.rotate_x(pi / 2.0)
    return mesh_from_geometry(temple, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wayfarer_glasses")

    frame_color = model.material("frame_black", rgba=(0.06, 0.06, 0.07, 1.0))
    hinge_color = model.material("hinge_metal", rgba=(0.38, 0.38, 0.40, 1.0))

    front = model.part("front_frame")
    front.visual(
        _build_front_shell_mesh(),
        material=frame_color,
        name="front_shell",
    )
    front.visual(
        Cylinder(radius=0.0022, length=0.0075),
        origin=Origin(xyz=(-HINGE_X, HINGE_Y, HINGE_Z + 0.0086)),
        material=hinge_color,
        name="left_upper_knuckle",
    )
    front.visual(
        Cylinder(radius=0.0022, length=0.0075),
        origin=Origin(xyz=(-HINGE_X, HINGE_Y, HINGE_Z - 0.0086)),
        material=hinge_color,
        name="left_lower_knuckle",
    )
    front.visual(
        Box((0.0035, 0.0030, 0.022)),
        origin=Origin(xyz=(-0.0724, HINGE_Y + 0.0001, HINGE_Z)),
        material=frame_color,
        name="left_hinge_pad",
    )
    front.visual(
        Cylinder(radius=0.0022, length=0.0075),
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z + 0.0086)),
        material=hinge_color,
        name="right_upper_knuckle",
    )
    front.visual(
        Cylinder(radius=0.0022, length=0.0075),
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z - 0.0086)),
        material=hinge_color,
        name="right_lower_knuckle",
    )
    front.visual(
        Box((0.0035, 0.0030, 0.022)),
        origin=Origin(xyz=(0.0724, HINGE_Y + 0.0001, HINGE_Z)),
        material=frame_color,
        name="right_hinge_pad",
    )

    left_temple = model.part("left_temple")
    left_temple.visual(
        Cylinder(radius=0.0021, length=0.0090),
        origin=Origin(),
        material=hinge_color,
        name="left_temple_barrel",
    )
    left_temple.visual(
        _build_temple_main_mesh("left_temple_main_mesh"),
        origin=Origin(xyz=(-0.0026, -0.0003, -0.0002)),
        material=frame_color,
        name="left_temple_main",
    )

    right_temple = model.part("right_temple")
    right_temple.visual(
        Cylinder(radius=0.0021, length=0.0090),
        origin=Origin(),
        material=hinge_color,
        name="right_temple_barrel",
    )
    right_temple.visual(
        _build_temple_main_mesh("right_temple_main_mesh"),
        origin=Origin(xyz=(0.0026, -0.0003, -0.0002)),
        material=frame_color,
        name="right_temple_main",
    )

    model.articulation(
        "front_to_left_temple",
        ArticulationType.REVOLUTE,
        parent=front,
        child=left_temple,
        origin=Origin(xyz=(-HINGE_X, HINGE_Y, HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "front_to_right_temple",
        ArticulationType.REVOLUTE,
        parent=front,
        child=right_temple,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_frame")
    left_temple = object_model.get_part("left_temple")
    right_temple = object_model.get_part("right_temple")
    left_hinge = object_model.get_articulation("front_to_left_temple")
    right_hinge = object_model.get_articulation("front_to_right_temple")

    left_upper = front.get_visual("left_upper_knuckle")
    left_lower = front.get_visual("left_lower_knuckle")
    right_upper = front.get_visual("right_upper_knuckle")
    right_lower = front.get_visual("right_lower_knuckle")
    left_barrel = left_temple.get_visual("left_temple_barrel")
    right_barrel = right_temple.get_visual("right_temple_barrel")

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0}):
        ctx.expect_gap(
            front,
            left_temple,
            axis="y",
            positive_elem="front_shell",
            negative_elem="left_temple_main",
            min_gap=0.0,
            max_gap=0.003,
            name="left temple sits just behind the front frame when open",
        )
        ctx.expect_gap(
            front,
            right_temple,
            axis="y",
            positive_elem="front_shell",
            negative_elem="right_temple_main",
            min_gap=0.0,
            max_gap=0.003,
            name="right temple sits just behind the front frame when open",
        )

        ctx.expect_overlap(
            front,
            left_temple,
            axes="xy",
            elem_a=left_upper,
            elem_b=left_barrel,
            min_overlap=0.003,
            name="left temple barrel aligns with upper front knuckle",
        )
        ctx.expect_overlap(
            front,
            left_temple,
            axes="xy",
            elem_a=left_lower,
            elem_b=left_barrel,
            min_overlap=0.003,
            name="left temple barrel aligns with lower front knuckle",
        )
        ctx.expect_gap(
            front,
            left_temple,
            axis="z",
            positive_elem=left_upper,
            negative_elem=left_barrel,
            min_gap=0.0,
            max_gap=0.004,
            name="left upper knuckle sits just above the temple barrel",
        )
        ctx.expect_gap(
            left_temple,
            front,
            axis="z",
            positive_elem=left_barrel,
            negative_elem=left_lower,
            min_gap=0.0,
            max_gap=0.004,
            name="left lower knuckle sits just below the temple barrel",
        )

        ctx.expect_overlap(
            front,
            right_temple,
            axes="xy",
            elem_a=right_upper,
            elem_b=right_barrel,
            min_overlap=0.003,
            name="right temple barrel aligns with upper front knuckle",
        )
        ctx.expect_overlap(
            front,
            right_temple,
            axes="xy",
            elem_a=right_lower,
            elem_b=right_barrel,
            min_overlap=0.003,
            name="right temple barrel aligns with lower front knuckle",
        )
        ctx.expect_gap(
            front,
            right_temple,
            axis="z",
            positive_elem=right_upper,
            negative_elem=right_barrel,
            min_gap=0.0,
            max_gap=0.004,
            name="right upper knuckle sits just above the temple barrel",
        )
        ctx.expect_gap(
            right_temple,
            front,
            axis="z",
            positive_elem=right_barrel,
            negative_elem=right_lower,
            min_gap=0.0,
            max_gap=0.004,
            name="right lower knuckle sits just below the temple barrel",
        )

    left_fold = left_hinge.motion_limits.upper if left_hinge.motion_limits is not None else 1.45
    right_fold = right_hinge.motion_limits.upper if right_hinge.motion_limits is not None else 1.45

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0}):
        left_open_aabb = ctx.part_element_world_aabb(left_temple, elem="left_temple_main")
        right_open_aabb = ctx.part_element_world_aabb(right_temple, elem="right_temple_main")

    with ctx.pose({left_hinge: left_fold, right_hinge: right_fold}):
        left_folded_aabb = ctx.part_element_world_aabb(left_temple, elem="left_temple_main")
        right_folded_aabb = ctx.part_element_world_aabb(right_temple, elem="right_temple_main")
        ctx.expect_gap(
            front,
            left_temple,
            axis="y",
            positive_elem="front_shell",
            negative_elem="left_temple_main",
            max_penetration=0.0,
            max_gap=0.004,
            name="left temple stays behind the front frame when folded",
        )
        ctx.expect_gap(
            front,
            right_temple,
            axis="y",
            positive_elem="front_shell",
            negative_elem="right_temple_main",
            max_penetration=0.0,
            max_gap=0.004,
            name="right temple stays behind the front frame when folded",
        )

    def _center_x(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][0] + aabb[1][0])

    left_open_x = _center_x(left_open_aabb)
    left_folded_x = _center_x(left_folded_aabb)
    right_open_x = _center_x(right_open_aabb)
    right_folded_x = _center_x(right_folded_aabb)

    ctx.check(
        "left temple folds inward toward the bridge",
        left_open_x is not None and left_folded_x is not None and left_folded_x > left_open_x + 0.025,
        details=f"open_x={left_open_x}, folded_x={left_folded_x}",
    )
    ctx.check(
        "right temple folds inward toward the bridge",
        right_open_x is not None and right_folded_x is not None and right_folded_x < right_open_x - 0.025,
        details=f"open_x={right_open_x}, folded_x={right_folded_x}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
