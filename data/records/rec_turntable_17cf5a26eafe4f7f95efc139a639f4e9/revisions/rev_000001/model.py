from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _ring_mesh(outer_radius: float, inner_radius: float, height: float, name: str):
    """A one-piece hollow bearing collar, extrudable in a low-cost mold."""
    ring = cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)
    return mesh_from_cadquery(ring, name, tolerance=0.0007, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_turntable")

    molded_black = model.material("molded_black", color=(0.025, 0.026, 0.024, 1.0))
    deck_gray = model.material("warm_gray_deck", color=(0.20, 0.20, 0.18, 1.0))
    rubber = model.material("matte_rubber", color=(0.005, 0.005, 0.004, 1.0))
    aluminum = model.material("brushed_aluminum", color=(0.72, 0.70, 0.66, 1.0))
    dark_metal = model.material("dark_metal", color=(0.08, 0.085, 0.085, 1.0))
    screw_metal = model.material("zinc_screw_heads", color=(0.48, 0.48, 0.44, 1.0))

    # One molded plinth carries every fixed feature: feet, bearing collars,
    # screw bosses, and snap lips.  Separate visuals are fused/contacted to
    # the same link so the manufacturing logic remains a single shell.
    plinth = model.part("plinth")
    plinth_profile = rounded_rect_profile(0.46, 0.36, 0.034, corner_segments=10)
    plinth_cavity = rounded_rect_profile(0.400, 0.300, 0.026, corner_segments=10)
    plinth.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(plinth_profile, [plinth_cavity], 0.046, center=False),
            "molded_plinth_skirt",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=molded_black,
        name="molded_skirt",
    )
    plinth.visual(
        mesh_from_geometry(ExtrudeGeometry.from_z0(plinth_profile, 0.006), "molded_plinth_top"),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=molded_black,
        name="molded_top",
    )
    plinth.visual(
        Box((0.392, 0.294, 0.003)),
        origin=Origin(xyz=(-0.004, 0.0, 0.0645)),
        material=deck_gray,
        name="flat_top_skin",
    )
    plinth.visual(
        Cylinder(radius=0.158, length=0.004),
        origin=Origin(xyz=(-0.075, 0.018, 0.0645)),
        material=deck_gray,
        name="platter_recess",
    )
    plinth.visual(
        _ring_mesh(0.033, 0.018, 0.016, "platter_bearing_collar"),
        origin=Origin(xyz=(-0.075, 0.018, 0.064)),
        material=dark_metal,
        name="platter_bearing",
    )
    plinth.visual(
        _ring_mesh(0.032, 0.011, 0.024, "tonearm_bearing_collar"),
        origin=Origin(xyz=(0.150, 0.072, 0.064)),
        material=dark_metal,
        name="tonearm_pedestal",
    )
    for idx, (x, y) in enumerate(((-0.176, -0.132), (0.176, -0.132), (-0.176, 0.132), (0.176, 0.132))):
        plinth.visual(
            Cylinder(radius=0.024, length=0.012),
            origin=Origin(xyz=(x, y, 0.006)),
            material=rubber,
            name=f"rubber_foot_{idx}",
        )
    for idx, (x, y) in enumerate(((0.122, 0.045), (0.178, 0.099))):
        plinth.visual(
            Cylinder(radius=0.0065, length=0.003),
            origin=Origin(xyz=(x, y, 0.0660)),
            material=screw_metal,
            name=f"pedestal_screw_{idx}",
        )
    plinth.visual(
        Box((0.180, 0.009, 0.010)),
        origin=Origin(xyz=(-0.010, -0.175, 0.059)),
        material=molded_black,
        name="front_snap_lip",
    )
    plinth.visual(
        Box((0.150, 0.009, 0.010)),
        origin=Origin(xyz=(0.020, 0.175, 0.059)),
        material=molded_black,
        name="rear_snap_lip",
    )

    # The platter is a single stamped/turned rotating part with an integral
    # hub sliding inside the fixed bearing collar and a small center spindle.
    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.145, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=aluminum,
        name="platter_disk",
    )
    platter.visual(
        mesh_from_geometry(
            TorusGeometry(0.145, 0.003, radial_segments=40, tubular_segments=12),
            "platter_rolled_rim",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=aluminum,
        name="rolled_rim",
    )
    platter.visual(
        Cylinder(radius=0.132, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=rubber,
        name="record_mat",
    )
    platter.visual(
        Cylinder(radius=0.014, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=dark_metal,
        name="platter_hub",
    )
    platter.visual(
        Cylinder(radius=0.004, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=screw_metal,
        name="center_spindle",
    )

    model.articulation(
        "platter_spin",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(-0.075, 0.018, 0.080)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    # The tonearm is another low-count subassembly: one vertical shaft/cap,
    # one tube, one molded headshell, and one counterweight clamped to the same
    # rotating stage.
    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.008, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=screw_metal,
        name="pivot_shaft",
    )
    tonearm.visual(
        Cylinder(radius=0.026, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_metal,
        name="pivot_cap",
    )
    tonearm.visual(
        Box((0.034, 0.018, 0.018)),
        origin=Origin(xyz=(-0.026, -0.004, 0.020)),
        material=dark_metal,
        name="arm_socket",
    )
    tonearm.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-0.024, -0.004, 0.030),
                    (-0.080, -0.026, 0.034),
                    (-0.155, -0.060, 0.032),
                    (-0.197, -0.082, 0.030),
                ],
                radius=0.0035,
                samples_per_segment=12,
                radial_segments=16,
                cap_ends=True,
            ),
            "tonearm_swept_tube",
        ),
        material=aluminum,
        name="arm_tube",
    )
    tonearm.visual(
        Box((0.040, 0.025, 0.006)),
        origin=Origin(xyz=(-0.208, -0.084, 0.027), rpy=(0.0, 0.0, -0.32)),
        material=molded_black,
        name="headshell",
    )
    tonearm.visual(
        Box((0.018, 0.012, 0.010)),
        origin=Origin(xyz=(-0.218, -0.087, 0.019), rpy=(0.0, 0.0, -0.32)),
        material=molded_black,
        name="cartridge",
    )
    tonearm.visual(
        Sphere(radius=0.0015),
        origin=Origin(xyz=(-0.222, -0.088, 0.013)),
        material=screw_metal,
        name="stylus_tip",
    )
    tonearm.visual(
        Cylinder(radius=0.005, length=0.036),
        origin=Origin(xyz=(0.020, 0.0, 0.026), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=screw_metal,
        name="counterweight_stem",
    )
    tonearm.visual(
        Cylinder(radius=0.014, length=0.052),
        origin=Origin(xyz=(0.052, 0.0, 0.026), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="counterweight",
    )

    model.articulation(
        "tonearm_swing",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=(0.150, 0.072, 0.088)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=1.2, lower=-0.38, upper=0.52),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm")
    platter_spin = object_model.get_articulation("platter_spin")
    tonearm_swing = object_model.get_articulation("tonearm_swing")

    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        positive_elem="platter_disk",
        negative_elem="platter_bearing",
        max_gap=0.0008,
        max_penetration=0.0,
        name="platter rests on bearing collar",
    )
    ctx.expect_within(
        platter,
        plinth,
        axes="xy",
        inner_elem="platter_hub",
        outer_elem="platter_bearing",
        margin=0.0005,
        name="platter hub is centered in bearing",
    )
    ctx.expect_overlap(
        platter,
        plinth,
        axes="z",
        elem_a="platter_hub",
        elem_b="platter_bearing",
        min_overlap=0.014,
        name="platter hub stays inserted in collar",
    )
    ctx.expect_gap(
        tonearm,
        plinth,
        axis="z",
        positive_elem="pivot_cap",
        negative_elem="tonearm_pedestal",
        max_gap=0.0008,
        max_penetration=0.0,
        name="tonearm cap sits on pedestal",
    )
    ctx.expect_within(
        tonearm,
        plinth,
        axes="xy",
        inner_elem="pivot_shaft",
        outer_elem="tonearm_pedestal",
        margin=0.0005,
        name="tonearm shaft is centered in pedestal",
    )
    ctx.expect_overlap(
        tonearm,
        plinth,
        axes="z",
        elem_a="pivot_shaft",
        elem_b="tonearm_pedestal",
        min_overlap=0.022,
        name="tonearm shaft remains captured",
    )
    ctx.expect_gap(
        tonearm,
        platter,
        axis="z",
        positive_elem="stylus_tip",
        negative_elem="record_mat",
        min_gap=0.001,
        max_gap=0.004,
        name="stylus clears the record mat",
    )

    rest_aabb = ctx.part_element_world_aabb(tonearm, elem="headshell")
    with ctx.pose({tonearm_swing: 0.48, platter_spin: 1.2}):
        swept_aabb = ctx.part_element_world_aabb(tonearm, elem="headshell")
        ctx.expect_gap(
            tonearm,
            platter,
            axis="z",
            positive_elem="headshell",
            negative_elem="record_mat",
            min_gap=0.006,
            name="swept headshell stays above platter",
        )

    def _center_xy(aabb):
        if aabb is None:
            return None
        mn, mx = aabb
        return ((mn[0] + mx[0]) * 0.5, (mn[1] + mx[1]) * 0.5)

    rest_xy = _center_xy(rest_aabb)
    swept_xy = _center_xy(swept_aabb)
    ctx.check(
        "tonearm pivot changes headshell position",
        rest_xy is not None
        and swept_xy is not None
        and math.hypot(swept_xy[0] - rest_xy[0], swept_xy[1] - rest_xy[1]) > 0.060,
        details=f"rest={rest_xy}, swept={swept_xy}",
    )

    return ctx.report()


object_model = build_object_model()
