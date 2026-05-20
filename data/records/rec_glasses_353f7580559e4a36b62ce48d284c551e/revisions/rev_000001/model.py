from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


FRAME_HALF_WIDTH = 0.074
HINGE_X = 0.087
HINGE_Y = 0.024


def _shield_half_height(x: float) -> float:
    """Tall in the center and clipped low at the wraparound corners."""
    t = min(1.0, abs(x) / FRAME_HALF_WIDTH)
    return 0.017 + 0.011 * (1.0 - t**1.7)


def _shield_y(x: float) -> float:
    """Horizontal wraparound curvature: center forward, corners swept back."""
    t = min(1.0, abs(x) / FRAME_HALF_WIDTH)
    return -0.006 + 0.022 * t * t


def _shield_lens_geometry() -> MeshGeometry:
    """Closed, thin single shield lens following the curved front frame."""
    nx = 34
    nz = 10
    thickness = 0.0022
    geom = MeshGeometry()
    front: list[list[int]] = []
    back: list[list[int]] = []

    for i in range(nx + 1):
        x = -FRAME_HALF_WIDTH + 2.0 * FRAME_HALF_WIDTH * i / nx
        half_h = _shield_half_height(x)
        y_mid = _shield_y(x)
        front_row: list[int] = []
        back_row: list[int] = []
        for j in range(nz + 1):
            v = -1.0 + 2.0 * j / nz
            z = v * half_h
            # A subtle spherical visor bulge prevents the shield from reading
            # as a flat sheet while retaining the horizontal wraparound.
            y_bulge = -0.0015 * (1.0 - (abs(v) ** 2.2)) * (1.0 - (abs(x) / FRAME_HALF_WIDTH) ** 2.0)
            front_row.append(geom.add_vertex(x, y_mid + y_bulge - thickness / 2.0, z))
            back_row.append(geom.add_vertex(x, y_mid + y_bulge + thickness / 2.0, z))
        front.append(front_row)
        back.append(back_row)

    for i in range(nx):
        for j in range(nz):
            a, b, c, d = front[i][j], front[i + 1][j], front[i + 1][j + 1], front[i][j + 1]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)
            a, b, c, d = back[i][j], back[i][j + 1], back[i + 1][j + 1], back[i + 1][j]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)

    # Close the top, bottom, and side lips of the thin lens.
    for i in range(nx):
        # bottom lip
        a, b, c, d = front[i][0], back[i][0], back[i + 1][0], front[i + 1][0]
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)
        # top lip
        a, b, c, d = front[i][nz], front[i + 1][nz], back[i + 1][nz], back[i][nz]
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)
    for j in range(nz):
        # left side lip
        a, b, c, d = front[0][j], front[0][j + 1], back[0][j + 1], back[0][j]
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)
        # right side lip
        a, b, c, d = front[nx][j], back[nx][j], back[nx][j + 1], front[nx][j + 1]
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)
    return geom


def _frame_rail_geometry() -> MeshGeometry:
    geom = MeshGeometry()
    top_points = []
    bottom_points = []
    for i in range(15):
        x = -FRAME_HALF_WIDTH + 2.0 * FRAME_HALF_WIDTH * i / 14
        top_points.append((x, _shield_y(x), _shield_half_height(x) + 0.0015))
        bottom_points.append((x, _shield_y(x), -_shield_half_height(x) - 0.0015))

    geom.merge(
        tube_from_spline_points(
            top_points,
            radius=0.0032,
            samples_per_segment=4,
            radial_segments=18,
            cap_ends=True,
        )
    )
    geom.merge(
        tube_from_spline_points(
            bottom_points,
            radius=0.0032,
            samples_per_segment=4,
            radial_segments=18,
            cap_ends=True,
        )
    )
    for sign in (-1.0, 1.0):
        x = sign * FRAME_HALF_WIDTH
        y = _shield_y(x)
        h = _shield_half_height(x)
        geom.merge(
            tube_from_spline_points(
                [(x, y, -h - 0.001), (x, y, h + 0.001)],
                radius=0.0034,
                samples_per_segment=4,
                radial_segments=18,
                cap_ends=True,
            )
        )
    return geom


def _nosepiece_geometry() -> MeshGeometry:
    geom = MeshGeometry()
    for sign in (-1.0, 1.0):
        geom.merge(
            tube_from_spline_points(
                [
                    (sign * 0.009, _shield_y(sign * 0.009) + 0.001, -0.023),
                    (sign * 0.010, 0.009, -0.027),
                    (sign * 0.010, 0.019, -0.031),
                ],
                radius=0.0021,
                samples_per_segment=5,
                radial_segments=14,
                cap_ends=True,
            )
        )
    geom.merge(
        tube_from_spline_points(
            [(-0.012, 0.017, -0.032), (0.0, 0.021, -0.034), (0.012, 0.017, -0.032)],
            radius=0.0018,
            samples_per_segment=6,
            radial_segments=14,
            cap_ends=True,
        )
    )
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wraparound_sport_glasses")

    frame_mat = model.material("satin_black", rgba=(0.005, 0.005, 0.006, 1.0))
    lens_mat = model.material("smoke_blue_lens", rgba=(0.10, 0.18, 0.24, 0.47))
    rubber_mat = model.material("matte_rubber", rgba=(0.018, 0.018, 0.016, 1.0))
    pin_mat = model.material("brushed_pin", rgba=(0.55, 0.56, 0.54, 1.0))

    front = model.part("front_frame")
    front.visual(
        mesh_from_geometry(_shield_lens_geometry(), "curved_shield_lens"),
        material=lens_mat,
        name="shield_lens",
    )
    front.visual(
        mesh_from_geometry(_frame_rail_geometry(), "curved_front_rim"),
        material=frame_mat,
        name="curved_rim",
    )
    front.visual(
        mesh_from_geometry(_nosepiece_geometry(), "integrated_nosepiece"),
        material=rubber_mat,
        name="nosepiece",
    )

    for sign, side_name in ((-1.0, "left"), (1.0, "right")):
        inboard = -sign
        hinge_x = sign * HINGE_X
        # Compact outer hinge block: an outboard spine connects upper and lower
        # clevis leaves, leaving a central slot for the temple pivot barrel.
        front.visual(
            Box((0.005, 0.019, 0.038)),
            origin=Origin(xyz=(hinge_x + sign * 0.0045, HINGE_Y, 0.0)),
            material=frame_mat,
            name=f"{side_name}_hinge_spine",
        )
        front.visual(
            Box((0.015, 0.020, 0.007)),
            origin=Origin(xyz=(hinge_x + inboard * 0.0045, HINGE_Y, 0.014)),
            material=frame_mat,
            name=f"{side_name}_hinge_top_lug",
        )
        front.visual(
            Box((0.015, 0.020, 0.007)),
            origin=Origin(xyz=(hinge_x + inboard * 0.0045, HINGE_Y, -0.014)),
            material=frame_mat,
            name=f"{side_name}_hinge_bottom_lug",
        )
        front.visual(
            Box((0.013, 0.009, 0.020)),
            origin=Origin(xyz=(sign * 0.079, 0.019, 0.0)),
            material=frame_mat,
            name=f"{side_name}_hinge_bridge",
        )
        front.visual(
            Cylinder(radius=0.0042, length=0.003),
            origin=Origin(xyz=(hinge_x, HINGE_Y, 0.0195)),
            material=pin_mat,
            name=f"{side_name}_pin_head",
        )

    def add_temple(side_name: str, sign: float) -> None:
        inboard = -sign
        temple = model.part(f"{side_name}_temple")
        temple.visual(
            Cylinder(radius=0.0038, length=0.016),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=pin_mat,
            name="pivot_barrel",
        )
        temple.visual(
            Box((0.008, 0.016, 0.010)),
            origin=Origin(xyz=(inboard * 0.004, 0.006, 0.0)),
            material=frame_mat,
            name="hinge_tongue",
        )
        temple.visual(
            Box((0.007, 0.120, 0.006)),
            origin=Origin(xyz=(inboard * 0.003, 0.073, 0.0005)),
            material=frame_mat,
            name="straight_arm",
        )
        temple.visual(
            Box((0.0085, 0.040, 0.0075)),
            origin=Origin(xyz=(inboard * 0.003, 0.132, -0.0015)),
            material=rubber_mat,
            name="ear_sleeve",
        )

        lower, upper = (-1.75, 0.0) if side_name == "left" else (0.0, 1.75)
        model.articulation(
            f"front_to_{side_name}_temple",
            ArticulationType.REVOLUTE,
            parent=front,
            child=temple,
            origin=Origin(xyz=(sign * HINGE_X, HINGE_Y, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.8, velocity=3.0, lower=lower, upper=upper),
        )

    add_temple("left", -1.0)
    add_temple("right", 1.0)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_frame")
    left = object_model.get_part("left_temple")
    right = object_model.get_part("right_temple")
    left_joint = object_model.get_articulation("front_to_left_temple")
    right_joint = object_model.get_articulation("front_to_right_temple")

    ctx.check(
        "two separate temple hinges",
        left_joint.motion_limits.lower < -1.6
        and left_joint.motion_limits.upper == 0.0
        and right_joint.motion_limits.lower == 0.0
        and right_joint.motion_limits.upper > 1.6,
        details=f"left={left_joint.motion_limits}, right={right_joint.motion_limits}",
    )

    for side_name, temple in (("left", left), ("right", right)):
        ctx.expect_gap(
            front,
            temple,
            axis="z",
            positive_elem=f"{side_name}_hinge_top_lug",
            negative_elem="pivot_barrel",
            min_gap=0.001,
            max_gap=0.004,
            name=f"{side_name} pivot is under top hinge lug",
        )
        ctx.expect_gap(
            temple,
            front,
            axis="z",
            positive_elem="pivot_barrel",
            negative_elem=f"{side_name}_hinge_bottom_lug",
            min_gap=0.001,
            max_gap=0.004,
            name=f"{side_name} pivot is above bottom hinge lug",
        )
        ctx.expect_overlap(
            front,
            temple,
            axes="xy",
            elem_a=f"{side_name}_hinge_top_lug",
            elem_b="pivot_barrel",
            min_overlap=0.004,
            name=f"{side_name} pivot remains captured in hinge block",
        )

    rest_left = ctx.part_element_world_aabb(left, elem="straight_arm")
    rest_right = ctx.part_element_world_aabb(right, elem="straight_arm")
    ctx.check(
        "temple arms project straight rearward at rest",
        rest_left is not None
        and rest_right is not None
        and rest_left[0][1] > HINGE_Y + 0.010
        and rest_right[0][1] > HINGE_Y + 0.010,
        details=f"left_aabb={rest_left}, right_aabb={rest_right}",
    )

    with ctx.pose({left_joint: -1.45, right_joint: 1.45}):
        folded_left = ctx.part_element_world_aabb(left, elem="straight_arm")
        folded_right = ctx.part_element_world_aabb(right, elem="straight_arm")
        ctx.check(
            "temple arms fold inward on independent vertical axes",
            rest_left is not None
            and rest_right is not None
            and folded_left is not None
            and folded_right is not None
            and folded_left[1][0] > rest_left[1][0] + 0.045
            and folded_right[0][0] < rest_right[0][0] - 0.045,
            details=f"folded_left={folded_left}, folded_right={folded_right}",
        )
        ctx.expect_overlap(
            front,
            left,
            axes="xy",
            elem_a="left_hinge_top_lug",
            elem_b="pivot_barrel",
            min_overlap=0.004,
            name="left folded pivot stays in block",
        )
        ctx.expect_overlap(
            front,
            right,
            axes="xy",
            elem_a="right_hinge_top_lug",
            elem_b="pivot_barrel",
            min_overlap=0.004,
            name="right folded pivot stays in block",
        )

    return ctx.report()


object_model = build_object_model()
