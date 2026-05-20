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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sample_catmull_rom_spline_2d,
    sweep_profile_along_spline,
)


LOWER_SHELL_HEIGHT = 0.096
LOWER_FLOOR_THICKNESS = 0.006
LID_WALL_HEIGHT = 0.040
LID_TOP_THICKNESS = 0.006
SEAM_Z = LOWER_SHELL_HEIGHT
HINGE_Y = -0.112
HANDLE_Y = 0.182
HANDLE_Z = 0.080


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _shift_profile(profile, *, dx: float = 0.0, dy: float = 0.0):
    return [(x + dx, y + dy) for x, y in profile]


def _smooth_closed_profile(points):
    return sample_catmull_rom_spline_2d(points, samples_per_segment=10, closed=True)


def _case_profiles():
    outer = _smooth_closed_profile(
        [
            (-0.400, -0.015),
            (-0.395, -0.086),
            (-0.290, -0.108),
            (-0.080, -0.116),
            (0.140, -0.116),
            (0.310, -0.108),
            (0.380, -0.060),
            (0.382, 0.000),
            (0.360, 0.050),
            (0.300, 0.095),
            (0.200, 0.145),
            (0.110, 0.168),
            (-0.010, 0.144),
            (-0.100, 0.150),
            (-0.210, 0.208),
            (-0.310, 0.198),
            (-0.385, 0.122),
            (-0.400, 0.040),
        ]
    )
    inner = _smooth_closed_profile(
        [
            (-0.385, -0.013),
            (-0.382, -0.074),
            (-0.286, -0.094),
            (-0.080, -0.100),
            (0.140, -0.100),
            (0.305, -0.094),
            (0.366, -0.055),
            (0.367, -0.003),
            (0.346, 0.036),
            (0.288, 0.078),
            (0.194, 0.124),
            (0.110, 0.145),
            (-0.010, 0.123),
            (-0.100, 0.128),
            (-0.200, 0.184),
            (-0.300, 0.176),
            (-0.370, 0.106),
            (-0.385, 0.031),
        ]
    )
    lining = _smooth_closed_profile(
        [
            (-0.372, -0.010),
            (-0.370, -0.066),
            (-0.278, -0.084),
            (-0.080, -0.088),
            (0.140, -0.088),
            (0.296, -0.084),
            (0.356, -0.049),
            (0.357, -0.002),
            (0.336, 0.030),
            (0.280, 0.069),
            (0.188, 0.112),
            (0.110, 0.132),
            (-0.010, 0.112),
            (-0.098, 0.117),
            (-0.192, 0.170),
            (-0.290, 0.162),
            (-0.358, 0.098),
            (-0.372, 0.026),
        ]
    )
    return outer, inner, lining


def _handle_path():
    return [
        (-0.120, 0.008, 0.000),
        (-0.118, 0.018, 0.007),
        (-0.105, 0.031, 0.014),
        (-0.072, 0.041, 0.020),
        (-0.028, 0.048, 0.022),
        (0.028, 0.048, 0.022),
        (0.072, 0.041, 0.020),
        (0.105, 0.031, 0.014),
        (0.118, 0.018, 0.007),
        (0.120, 0.008, 0.000),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="violin_case")

    shell_black = model.material("shell_black", rgba=(0.10, 0.10, 0.11, 1.0))
    seam_black = model.material("seam_black", rgba=(0.06, 0.06, 0.07, 1.0))
    plush_blue = model.material("plush_blue", rgba=(0.12, 0.18, 0.30, 1.0))
    plush_shadow = model.material("plush_shadow", rgba=(0.09, 0.13, 0.22, 1.0))
    leather_handle = model.material("leather_handle", rgba=(0.16, 0.10, 0.06, 1.0))
    hardware = model.material("hardware", rgba=(0.66, 0.68, 0.72, 1.0))

    outer_profile, inner_profile, lining_profile = _case_profiles()
    lid_outer_profile = _shift_profile(outer_profile, dy=-HINGE_Y)
    lid_inner_profile = _shift_profile(inner_profile, dy=-HINGE_Y)
    lid_lining_profile = _shift_profile(lining_profile, dy=-HINGE_Y)

    lower_shell = model.part("lower_shell")
    lower_shell.inertial = Inertial.from_geometry(
        Box((0.82, 0.34, 0.12)),
        mass=4.8,
        origin=Origin(xyz=(-0.01, 0.02, 0.06)),
    )
    lower_shell.visual(
        _mesh(
            "lower_wall",
            ExtrudeWithHolesGeometry(
                outer_profile,
                [inner_profile],
                LOWER_SHELL_HEIGHT,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, LOWER_SHELL_HEIGHT * 0.5)),
        material=shell_black,
        name="lower_wall",
    )
    lower_shell.visual(
        _mesh(
            "lower_floor",
            ExtrudeGeometry(
                outer_profile,
                LOWER_FLOOR_THICKNESS,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, LOWER_FLOOR_THICKNESS * 0.5)),
        material=shell_black,
        name="lower_floor",
    )
    lower_shell.visual(
        Box((0.72, 0.010, 0.020)),
        origin=Origin(xyz=(-0.01, -0.110, 0.070)),
        material=seam_black,
        name="rear_hinge_strip",
    )
    for x in (-0.120, 0.120):
        lower_shell.visual(
            Box((0.024, 0.018, 0.026)),
            origin=Origin(xyz=(x, HANDLE_Y - 0.014, HANDLE_Z)),
            material=hardware,
            name=f"handle_bracket_{'left' if x < 0.0 else 'right'}",
        )
        lower_shell.visual(
            Cylinder(radius=0.0025, length=0.014),
            origin=Origin(xyz=(x, HANDLE_Y - 0.010, HANDLE_Z), rpy=(0.0, pi / 2.0, 0.0)),
            material=hardware,
            name=f"handle_pivot_{'left' if x < 0.0 else 'right'}",
        )

    instrument_bed = model.part("instrument_bed")
    instrument_bed.inertial = Inertial.from_geometry(
        Box((0.76, 0.28, 0.05)),
        mass=1.0,
        origin=Origin(xyz=(-0.01, 0.02, 0.025)),
    )
    instrument_bed.visual(
        _mesh(
            "bed_base",
            ExtrudeGeometry(
                lining_profile,
                0.008,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=plush_blue,
        name="bed_base",
    )
    instrument_bed.visual(
        Box((0.19, 0.13, 0.024)),
        origin=Origin(xyz=(-0.255, 0.050, 0.020)),
        material=plush_shadow,
        name="lower_bout_pad",
    )
    instrument_bed.visual(
        Box((0.17, 0.10, 0.022)),
        origin=Origin(xyz=(-0.030, 0.028, 0.019)),
        material=plush_shadow,
        name="upper_bout_pad",
    )
    instrument_bed.visual(
        Box((0.15, 0.050, 0.024)),
        origin=Origin(xyz=(0.215, 0.015, 0.020)),
        material=plush_shadow,
        name="neck_support",
    )
    instrument_bed.visual(
        Box((0.085, 0.055, 0.028)),
        origin=Origin(xyz=(-0.330, 0.000, 0.022)),
        material=plush_shadow,
        name="accessory_block",
    )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((0.82, 0.34, 0.07)),
        mass=2.6,
        origin=Origin(xyz=(-0.01, 0.13, 0.035)),
    )
    lid.visual(
        _mesh(
            "lid_wall",
            ExtrudeWithHolesGeometry(
                lid_outer_profile,
                [lid_inner_profile],
                LID_WALL_HEIGHT,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, LID_WALL_HEIGHT * 0.5)),
        material=shell_black,
        name="lid_wall",
    )
    lid.visual(
        _mesh(
            "lid_top",
            ExtrudeGeometry(
                lid_outer_profile,
                LID_TOP_THICKNESS,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, LID_WALL_HEIGHT + LID_TOP_THICKNESS * 0.5)),
        material=shell_black,
        name="lid_top",
    )
    lid.visual(
        _mesh(
            "lid_lining",
            ExtrudeGeometry(
                lid_lining_profile,
                0.004,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, LID_WALL_HEIGHT - 0.002)),
        material=plush_blue,
        name="lid_lining",
    )
    lid.visual(
        Box((0.17, 0.070, 0.014)),
        origin=Origin(xyz=(-0.225, 0.165, 0.029)),
        material=plush_shadow,
        name="lid_accessory_pad",
    )

    handle = model.part("handle")
    handle.inertial = Inertial.from_geometry(
        Box((0.27, 0.06, 0.05)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.025, 0.010)),
    )
    handle.visual(
        _mesh(
            "handle_body",
            sweep_profile_along_spline(
                _handle_path(),
                profile=rounded_rect_profile(0.018, 0.012, radius=0.0035),
                samples_per_segment=12,
                cap_profile=True,
            ),
        ),
        material=leather_handle,
        name="handle_body",
    )
    for x in (-0.120, 0.120):
        handle.visual(
            Cylinder(radius=0.006, length=0.020),
            origin=Origin(xyz=(x, 0.000, 0.000), rpy=(0.0, pi / 2.0, 0.0)),
            material=hardware,
            name=f"handle_cap_{'left' if x < 0.0 else 'right'}",
        )

    model.articulation(
        "bed_mount",
        ArticulationType.FIXED,
        parent=lower_shell,
        child=instrument_bed,
        origin=Origin(xyz=(0.0, 0.0, LOWER_FLOOR_THICKNESS)),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, SEAM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.8, lower=0.0, upper=1.25),
    )
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=handle,
        origin=Origin(xyz=(0.0, HANDLE_Y, HANDLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_shell = object_model.get_part("lower_shell")
    instrument_bed = object_model.get_part("instrument_bed")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle")
    lid_hinge = object_model.get_articulation("lid_hinge")
    handle_pivot = object_model.get_articulation("handle_pivot")

    ctx.expect_contact(
        instrument_bed,
        lower_shell,
        elem_a="bed_base",
        elem_b="lower_floor",
        name="instrument bed sits on the lower shell floor",
    )
    ctx.expect_within(
        instrument_bed,
        lower_shell,
        axes="xy",
        inner_elem="bed_base",
        outer_elem="lower_wall",
        margin=0.0,
        name="instrument bed stays inside the shell footprint",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            lower_shell,
            axis="z",
            positive_elem="lid_wall",
            negative_elem="lower_wall",
            max_gap=0.0015,
            max_penetration=0.0,
            name="lid closes flush onto the lower shell rim",
        )
        ctx.expect_overlap(
            lid,
            lower_shell,
            axes="xy",
            elem_a="lid_top",
            elem_b="lower_wall",
            min_overlap=0.22,
            name="lid covers the lower shell footprint when closed",
        )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.10}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward around the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.09,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    closed_handle_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({handle_pivot: 1.10}):
        open_handle_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "handle rotates outward and upward on its pivots",
        closed_handle_aabb is not None
        and open_handle_aabb is not None
        and open_handle_aabb[1][2] > closed_handle_aabb[1][2] + 0.03,
        details=f"closed={closed_handle_aabb}, open={open_handle_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
