from __future__ import annotations

import math

import cadquery as cq
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
    mesh_from_cadquery,
)


CASE_HEIGHT = 0.024
STEM_WIDTH = 0.006
HOLE_WIDTH = 0.010
GUIDE_WALL = 0.0024
GUIDE_HEIGHT = 0.006


def _rotated_offset(x: float, y: float, yaw: float) -> tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (x * c - y * s, x * s + y * c)


def _regular_key_specs() -> list[dict[str, float]]:
    specs: list[dict[str, float]] = []
    pitch_x = 0.021
    pitch_y = 0.0215
    for side, center_x, yaw in (
        ("left", -0.096, math.radians(-10.0)),
        ("right", 0.096, math.radians(10.0)),
    ):
        for row in range(4):
            for col in range(5):
                lx = (col - 2) * pitch_x
                ly = -0.034 + row * pitch_y
                ox, oy = _rotated_offset(lx, ly, yaw)
                specs.append(
                    {
                        "kind": "regular",
                        "side": side,
                        "x": center_x + ox,
                        "y": oy,
                        "yaw": yaw,
                        "width": 0.0178,
                        "depth": 0.0178,
                    }
                )
    return specs


def _thumb_key_specs() -> list[dict[str, float]]:
    return [
        {
            "kind": "thumb",
            "side": "left",
            "x": -0.076,
            "y": -0.061,
            "yaw": math.radians(-14.0),
            "width": 0.052,
            "depth": 0.018,
        },
        {
            "kind": "thumb",
            "side": "right",
            "x": 0.076,
            "y": -0.061,
            "yaw": math.radians(14.0),
            "width": 0.052,
            "depth": 0.018,
        },
    ]


def _all_key_specs() -> list[dict[str, float]]:
    return _regular_key_specs() + _thumb_key_specs()


def _stem_offsets(spec: dict[str, float]) -> list[tuple[float, float]]:
    if spec["kind"] == "thumb":
        return [(-0.018, 0.0), (0.0, 0.0), (0.018, 0.0)]
    return [(0.0, 0.0)]


def _keycap_shape(width: float, depth: float, height: float = 0.008) -> cq.Workplane:
    # A slightly tapered cap reads much more like a real key than a plain block.
    return (
        cq.Workplane("XY")
        .rect(width, depth)
        .workplane(offset=height)
        .rect(width * 0.82, depth * 0.82)
        .loft(combine=True)
    )


def _keyboard_case_shape() -> cq.Workplane:
    # One continuous Alice-style housing: broad rear edge, clipped outer corners,
    # and a V-shaped central front notch between the split thumb area.
    footprint = [
        (-0.235, -0.084),
        (-0.071, -0.084),
        (-0.039, -0.047),
        (-0.016, -0.040),
        (0.016, -0.040),
        (0.039, -0.047),
        (0.071, -0.084),
        (0.235, -0.084),
        (0.224, 0.098),
        (-0.224, 0.098),
    ]
    body = (
        cq.Workplane("XY")
        .polyline(footprint)
        .close()
        .extrude(CASE_HEIGHT)
        .edges("|Z")
        .fillet(0.006)
    )

    # Through-slots for the switch stems keep the plungers mechanically credible
    # instead of having them disappear into a solid plate.
    for spec in _all_key_specs():
        for ox, oy in _stem_offsets(spec):
            dx, dy = _rotated_offset(ox, oy, spec["yaw"])
            cutter = (
                cq.Workplane("XY")
                .box(HOLE_WIDTH, HOLE_WIDTH, 0.070)
                .rotate((0, 0, 0), (0, 0, 1), math.degrees(spec["yaw"]))
                .translate((spec["x"] + dx, spec["y"] + dy, CASE_HEIGHT / 2.0))
            )
            body = body.cut(cutter)
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="alice_ergonomic_keyboard")

    case_mat = model.material("warm_graphite", rgba=(0.055, 0.058, 0.064, 1.0))
    key_mat = model.material("warm_ivory_caps", rgba=(0.78, 0.75, 0.68, 1.0))
    stem_mat = model.material("dark_switch_stems", rgba=(0.025, 0.027, 0.030, 1.0))
    rubber_mat = model.material("soft_black_rubber", rgba=(0.010, 0.011, 0.012, 1.0))
    metal_mat = model.material("dark_hinge_metal", rgba=(0.11, 0.11, 0.12, 1.0))

    case = model.part("case")
    case.visual(
        mesh_from_cadquery(_keyboard_case_shape(), "continuous_notched_case", tolerance=0.0008),
        material=case_mat,
        name="notched_shell",
    )
    case.visual(
        Box((0.390, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.087, CASE_HEIGHT + 0.004)),
        material=case_mat,
        name="rear_lip",
    )
    for idx, sx in enumerate((-1.0, 1.0)):
        case.visual(
            Cylinder(radius=0.0032, length=0.0012),
            origin=Origin(xyz=(sx * 0.193, 0.070, CASE_HEIGHT + 0.0006)),
            material=metal_mat,
            name=f"rear_screw_{idx}",
        )
        case.visual(
            Cylinder(radius=0.0032, length=0.0012),
            origin=Origin(xyz=(sx * 0.178, -0.065, CASE_HEIGHT + 0.0006)),
            material=metal_mat,
            name=f"front_screw_{idx}",
        )

    # Fixed switch guide collars are part of the case.  Their inner faces touch
    # the moving plungers so the keys read as captured sliders rather than
    # unsupported caps hovering above a plate.
    for key_idx, spec in enumerate(_all_key_specs()):
        for stem_idx, (ox, oy) in enumerate(_stem_offsets(spec)):
            sx, sy = _rotated_offset(ox, oy, spec["yaw"])
            cx = spec["x"] + sx
            cy = spec["y"] + sy
            guide_z = CASE_HEIGHT + GUIDE_HEIGHT / 2.0 - 0.00025
            wall_specs = [
                ("x0", -(STEM_WIDTH / 2.0 + GUIDE_WALL / 2.0), 0.0, GUIDE_WALL, STEM_WIDTH),
                ("x1", STEM_WIDTH / 2.0 + GUIDE_WALL / 2.0, 0.0, GUIDE_WALL, STEM_WIDTH),
                ("y0", 0.0, -(STEM_WIDTH / 2.0 + GUIDE_WALL / 2.0), STEM_WIDTH + 2.0 * GUIDE_WALL, GUIDE_WALL),
                ("y1", 0.0, STEM_WIDTH / 2.0 + GUIDE_WALL / 2.0, STEM_WIDTH + 2.0 * GUIDE_WALL, GUIDE_WALL),
            ]
            for side_name, lx, ly, sx_size, sy_size in wall_specs:
                wx, wy = _rotated_offset(lx, ly, spec["yaw"])
                case.visual(
                    Box((sx_size, sy_size, GUIDE_HEIGHT)),
                    origin=Origin(
                        xyz=(cx + wx, cy + wy, guide_z),
                        rpy=(0.0, 0.0, spec["yaw"]),
                    ),
                    material=stem_mat,
                    name=f"guide_{key_idx}_{stem_idx}_{side_name}",
                )

    regular_cap_mesh = mesh_from_cadquery(
        _keycap_shape(0.0178, 0.0178), "regular_tapered_keycap", tolerance=0.0006
    )
    thumb_cap_mesh = mesh_from_cadquery(
        _keycap_shape(0.052, 0.018), "thumb_tapered_keycap", tolerance=0.0006
    )

    for idx, spec in enumerate(_all_key_specs()):
        key = model.part(f"key_{idx}")
        cap_mesh = thumb_cap_mesh if spec["kind"] == "thumb" else regular_cap_mesh
        key.visual(
            cap_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.008)),
            material=key_mat,
            name="cap",
        )
        for stem_idx, (ox, oy) in enumerate(_stem_offsets(spec)):
            key.visual(
                Box((STEM_WIDTH, STEM_WIDTH, 0.021)),
                origin=Origin(xyz=(ox, oy, -0.0015)),
                material=stem_mat,
                name="stem" if stem_idx == 0 else f"stabilizer_{stem_idx}",
            )
        model.articulation(
            f"case_to_key_{idx}",
            ArticulationType.PRISMATIC,
            parent=case,
            child=key,
            origin=Origin(xyz=(spec["x"], spec["y"], CASE_HEIGHT), rpy=(0.0, 0.0, spec["yaw"])),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2.5, velocity=0.35, lower=0.0, upper=0.004),
        )

    hinge_y = 0.083
    hinge_z = -0.004
    for idx, sx in enumerate((-1.0, 1.0)):
        x = sx * 0.168
        case.visual(
            Cylinder(radius=0.004, length=0.062),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name=f"hinge_barrel_{idx}",
        )
        case.visual(
            Box((0.067, 0.010, 0.004)),
            origin=Origin(xyz=(x, hinge_y, -0.0015)),
            material=case_mat,
            name=f"hinge_leaf_{idx}",
        )

        foot = model.part(f"rear_foot_{idx}")
        foot.visual(
            Box((0.060, 0.060, 0.007)),
            origin=Origin(xyz=(0.0, -0.035, -0.009)),
            material=rubber_mat,
            name="rubber_pad",
        )
        foot.visual(
            Box((0.055, 0.009, 0.005)),
            origin=Origin(xyz=(0.0, -0.004, -0.0065)),
            material=metal_mat,
            name="hinge_tab",
        )
        model.articulation(
            f"case_to_rear_foot_{idx}",
            ArticulationType.REVOLUTE,
            parent=case,
            child=foot,
            origin=Origin(xyz=(x, hinge_y, hinge_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=1.5, lower=0.0, upper=1.05),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    sample_key = object_model.get_part("key_0")
    sample_key_joint = object_model.get_articulation("case_to_key_0")
    rest_key_pos = ctx.part_world_position(sample_key)
    with ctx.pose({sample_key_joint: 0.004}):
        pressed_key_pos = ctx.part_world_position(sample_key)
    ctx.check(
        "sample key plunges downward",
        rest_key_pos is not None
        and pressed_key_pos is not None
        and pressed_key_pos[2] < rest_key_pos[2] - 0.003,
        details=f"rest={rest_key_pos}, pressed={pressed_key_pos}",
    )

    for idx in (0, 1):
        foot = object_model.get_part(f"rear_foot_{idx}")
        foot_joint = object_model.get_articulation(f"case_to_rear_foot_{idx}")
        folded_aabb = ctx.part_world_aabb(foot)
        with ctx.pose({foot_joint: 1.05}):
            deployed_aabb = ctx.part_world_aabb(foot)
        ctx.check(
            f"rear tent foot {idx} swings below the case",
            folded_aabb is not None
            and deployed_aabb is not None
            and deployed_aabb[0][2] < folded_aabb[0][2] - 0.025,
            details=f"folded={folded_aabb}, deployed={deployed_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
