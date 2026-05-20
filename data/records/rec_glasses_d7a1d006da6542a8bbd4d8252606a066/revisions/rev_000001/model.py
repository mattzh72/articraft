from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
    ExtrudeGeometry,
    tube_from_spline_points,
)


def _tube(points, radius, name, *, radial_segments=16):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=10,
            radial_segments=radial_segments,
            cap_ends=True,
        ),
        name,
    )


def _ellipsoid_mesh(size, name):
    sx, sy, sz = size
    return mesh_from_geometry(
        SphereGeometry(1.0, width_segments=28, height_segments=16).scale(
            sx * 0.5, sy * 0.5, sz * 0.5
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="half_rim_reading_glasses")

    metal = model.material("polished_warm_wire", rgba=(0.73, 0.63, 0.45, 1.0))
    lens_mat = model.material("pale_transparent_lens", rgba=(0.78, 0.92, 1.0, 0.36))
    pad_mat = model.material("soft_clear_silicone", rgba=(0.92, 0.96, 0.94, 0.68))
    dark_tip = model.material("dark_temple_sleeve", rgba=(0.06, 0.05, 0.045, 1.0))

    frame = model.part("brow_frame")

    # Thin clear lenses sit below a continuous upper brow wire.  The lenses are
    # part of the fixed front frame because this half-rim style carries them in
    # the brow and lower support wires rather than in a separate moving carrier.
    lens_profile = superellipse_profile(0.052, 0.034, exponent=2.35, segments=60)
    lens_mesh = mesh_from_geometry(ExtrudeGeometry(lens_profile, 0.0016), "lens_oval")
    for side, x in (("left", -0.033), ("right", 0.033)):
        frame.visual(
            lens_mesh,
            origin=Origin(xyz=(x, -0.0004, -0.004), rpy=(-pi / 2, 0.0, 0.0)),
            material=lens_mat,
            name=f"{side}_lens",
        )

    # Continuous upper brow/bridge bar.
    frame.visual(
        _tube(
            [
                (-0.071, -0.0005, 0.009),
                (-0.055, -0.0007, 0.016),
                (-0.020, -0.0005, 0.016),
                (0.000, -0.0005, 0.020),
                (0.020, -0.0005, 0.016),
                (0.055, -0.0007, 0.016),
                (0.071, -0.0005, 0.009),
            ],
            0.0012,
            "continuous_brow_bar",
            radial_segments=18,
        ),
        material=metal,
        name="brow_bar",
    )

    # A small arched bridge below the brow gives the center a realistic saddle.
    frame.visual(
        _tube(
            [
                (-0.014, -0.0006, 0.011),
                (-0.007, -0.0012, 0.004),
                (0.000, -0.0015, 0.002),
                (0.007, -0.0012, 0.004),
                (0.014, -0.0006, 0.011),
            ],
            0.00095,
            "arched_bridge",
        ),
        material=metal,
        name="bridge_arch",
    )

    # Lower half-rim support wires under each lens plus short posts tying them
    # back to the upper brow bar.
    lower_specs = {
        "left": [
            (-0.060, -0.0012, -0.006),
            (-0.050, -0.0014, -0.017),
            (-0.033, -0.0014, -0.021),
            (-0.017, -0.0012, -0.014),
            (-0.011, -0.0010, -0.006),
        ],
        "right": [
            (0.060, -0.0012, -0.006),
            (0.050, -0.0014, -0.017),
            (0.033, -0.0014, -0.021),
            (0.017, -0.0012, -0.014),
            (0.011, -0.0010, -0.006),
        ],
    }
    for side, points in lower_specs.items():
        frame.visual(
            _tube(points, 0.00078, f"{side}_lower_support"),
            material=metal,
            name=f"{side}_lower_support",
        )
    for side, sx in (("left", -1.0), ("right", 1.0)):
        frame.visual(
            _tube(
                [
                    (sx * 0.071, -0.0005, 0.009),
                    (sx * 0.064, -0.0010, 0.002),
                    (sx * 0.060, -0.0012, -0.006),
                ],
                0.00082,
                f"{side}_outer_drop",
            ),
            material=metal,
            name=f"{side}_outer_drop",
        )
        frame.visual(
            _tube(
                [
                    (sx * 0.020, -0.0005, 0.016),
                    (sx * 0.014, -0.0006, 0.011),
                    (sx * 0.012, -0.0009, 0.003),
                    (sx * 0.011, -0.0010, -0.006),
                ],
                0.00072,
                f"{side}_inner_drop",
            ),
            material=metal,
            name=f"{side}_inner_drop",
        )

    # Nose-pad carrier wires rooted at the bridge.  The pads themselves are
    # revolute children attached at the tiny pivot ends.
    for side, sx in (("left", -1.0), ("right", 1.0)):
        frame.visual(
            _tube(
                [
                    (sx * 0.007, -0.0012, 0.004),
                    (sx * 0.010, 0.0040, -0.002),
                    (sx * 0.012, 0.0065, -0.007),
                ],
                0.00062,
                f"{side}_nose_arm",
                radial_segments=12,
            ),
            material=metal,
            name=f"{side}_nose_arm",
        )
        frame.visual(
            Cylinder(radius=0.0011, length=0.0038),
            origin=Origin(
                xyz=(sx * 0.012, 0.00485, -0.007),
                rpy=(-pi / 2, 0.0, 0.0),
            ),
            material=metal,
            name=f"{side}_pad_fork",
        )

    # Side hinge leaves and fixed knuckles.  The gap between fixed knuckles
    # captures the moving temple knuckle so each temple remains clipped to the
    # frame edge during folding.
    for side, sx in (("left", -1.0), ("right", 1.0)):
        frame.visual(
            Box((0.0034, 0.0022, 0.018)),
            origin=Origin(xyz=(sx * 0.071, 0.0005, 0.0095)),
            material=metal,
            name=f"{side}_hinge_leaf",
        )
        frame.visual(
            Box((0.0032, 0.0040, 0.0042)),
            origin=Origin(xyz=(sx * 0.071, 0.0025, 0.0143)),
            material=metal,
            name=f"{side}_hinge_upper_tab",
        )
        frame.visual(
            Box((0.0032, 0.0040, 0.0042)),
            origin=Origin(xyz=(sx * 0.071, 0.0025, 0.0047)),
            material=metal,
            name=f"{side}_hinge_lower_tab",
        )
        frame.visual(
            Cylinder(radius=0.00205, length=0.0050),
            origin=Origin(xyz=(sx * 0.071, 0.0042, 0.0143)),
            material=metal,
            name=f"{side}_hinge_upper",
        )
        frame.visual(
            Cylinder(radius=0.00205, length=0.0050),
            origin=Origin(xyz=(sx * 0.071, 0.0042, 0.0047)),
            material=metal,
            name=f"{side}_hinge_lower",
        )

    def build_temple(side: str, sx: float):
        temple = model.part(f"{side}_temple")
        temple.visual(
            Cylinder(radius=0.00172, length=0.0046),
            origin=Origin(),
            material=metal,
            name="hinge_knuckle",
        )
        temple.visual(
            _tube(
                [
                    (0.0, 0.0015, 0.0),
                    (0.0, 0.0040, 0.0),
                    (sx * 0.0038, 0.014, -0.0005),
                    (sx * 0.0065, 0.050, -0.0020),
                    (sx * 0.0060, 0.098, -0.0060),
                    (sx * 0.0030, 0.126, -0.0170),
                    (0.0, 0.140, -0.0210),
                ],
                0.00095,
                f"{side}_temple_wire",
                radial_segments=14,
            ),
            material=metal,
            name="temple_wire",
        )
        temple.visual(
            Box((0.0042, 0.020, 0.0035)),
            origin=Origin(xyz=(sx * 0.0010, 0.134, -0.0210), rpy=(0.20, 0.0, 0.0)),
            material=dark_tip,
            name="ear_sleeve",
        )
        return temple

    left_temple = build_temple("left", -1.0)
    right_temple = build_temple("right", 1.0)

    model.articulation(
        "left_temple_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_temple,
        origin=Origin(xyz=(-0.071, 0.0042, 0.0095)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=3.0, lower=0.0, upper=1.82),
    )
    model.articulation(
        "right_temple_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_temple,
        origin=Origin(xyz=(0.071, 0.0042, 0.0095)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=3.0, lower=0.0, upper=1.82),
    )

    def build_pad(side: str, sx: float):
        pad = model.part(f"{side}_nose_pad")
        pad.visual(
            _tube(
                [
                    (0.0, 0.00025, 0.0),
                    (sx * 0.0022, 0.0030, -0.0045),
                    (sx * 0.0048, 0.0060, -0.0110),
                ],
                0.00062,
                f"{side}_pad_stem",
                radial_segments=12,
            ),
            material=metal,
            name="pad_stem",
        )
        pad.visual(
            _ellipsoid_mesh((0.0064, 0.0023, 0.0145), f"{side}_silicone_pad"),
            origin=Origin(xyz=(sx * 0.0048, 0.0060, -0.0110)),
            material=pad_mat,
            name="silicone_pad",
        )
        return pad

    left_pad = build_pad("left", 1.0)
    right_pad = build_pad("right", -1.0)

    model.articulation(
        "left_pad_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_pad,
        origin=Origin(xyz=(-0.012, 0.0065, -0.007)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=2.0, lower=-0.45, upper=0.45),
    )
    model.articulation(
        "right_pad_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_pad,
        origin=Origin(xyz=(0.012, 0.0065, -0.007)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=2.0, lower=-0.45, upper=0.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("brow_frame")
    left_temple = object_model.get_part("left_temple")
    right_temple = object_model.get_part("right_temple")
    left_pad = object_model.get_part("left_nose_pad")
    right_pad = object_model.get_part("right_nose_pad")
    left_hinge = object_model.get_articulation("left_temple_hinge")
    right_hinge = object_model.get_articulation("right_temple_hinge")
    left_pivot = object_model.get_articulation("left_pad_pivot")
    right_pivot = object_model.get_articulation("right_pad_pivot")

    # The temple knuckle is retained between two fixed hinge barrels at the
    # frame edge.  These checks prove attachment without relying on penetration.
    for side, temple in (("left", left_temple), ("right", right_temple)):
        ctx.expect_overlap(
            frame,
            temple,
            axes="xy",
            min_overlap=0.0015,
            elem_a=f"{side}_hinge_upper",
            elem_b="hinge_knuckle",
            name=f"{side} temple knuckle coaxial with upper barrel",
        )
        ctx.expect_gap(
            frame,
            temple,
            axis="z",
            positive_elem=f"{side}_hinge_upper",
            negative_elem="hinge_knuckle",
            min_gap=0.0,
            max_gap=0.0002,
            name=f"{side} upper hinge clearance",
        )
        ctx.expect_gap(
            temple,
            frame,
            axis="z",
            positive_elem="hinge_knuckle",
            negative_elem=f"{side}_hinge_lower",
            min_gap=0.0,
            max_gap=0.0002,
            name=f"{side} lower hinge clearance",
        )

    def _coord(vec, index):
        if hasattr(vec, "x"):
            return (vec.x, vec.y, vec.z)[index]
        return vec[index]

    def _aabb_center(bounds, index):
        return (_coord(bounds[0], index) + _coord(bounds[1], index)) * 0.5

    left_rest = ctx.part_element_world_aabb(left_temple, elem="temple_wire")
    right_rest = ctx.part_element_world_aabb(right_temple, elem="temple_wire")
    with ctx.pose({left_hinge: 1.45, right_hinge: 1.45}):
        left_folded = ctx.part_element_world_aabb(left_temple, elem="temple_wire")
        right_folded = ctx.part_element_world_aabb(right_temple, elem="temple_wire")

    ctx.check(
        "temples fold inward",
        left_rest is not None
        and right_rest is not None
        and left_folded is not None
        and right_folded is not None
        and _aabb_center(left_folded, 0) > _aabb_center(left_rest, 0) + 0.060
        and _aabb_center(right_folded, 0) < _aabb_center(right_rest, 0) - 0.060,
        details=f"left_rest={left_rest}, left_folded={left_folded}, right_rest={right_rest}, right_folded={right_folded}",
    )

    left_pad_rest = ctx.part_element_world_aabb(left_pad, elem="silicone_pad")
    right_pad_rest = ctx.part_element_world_aabb(right_pad, elem="silicone_pad")
    with ctx.pose({left_pivot: 0.35, right_pivot: -0.35}):
        left_pad_rotated = ctx.part_element_world_aabb(left_pad, elem="silicone_pad")
        right_pad_rotated = ctx.part_element_world_aabb(right_pad, elem="silicone_pad")
    ctx.check(
        "nose pads swivel on bridge pivots",
        left_pad_rest is not None
        and right_pad_rest is not None
        and left_pad_rotated is not None
        and right_pad_rotated is not None
        and abs(_aabb_center(left_pad_rotated, 0) - _aabb_center(left_pad_rest, 0)) > 0.002
        and abs(_aabb_center(right_pad_rotated, 0) - _aabb_center(right_pad_rest, 0)) > 0.002,
        details=f"left_rest={left_pad_rest}, left_rotated={left_pad_rotated}, right_rest={right_pad_rest}, right_rotated={right_pad_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
