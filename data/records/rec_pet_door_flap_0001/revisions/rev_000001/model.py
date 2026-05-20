from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
def _make_material(name: str, rgba: tuple[float, float, float, float]) -> Material | None:
    for key in ("color", "rgba"):
        try:
            return Material(name=name, **{key: rgba})
        except TypeError:
            continue
    try:
        material = Material(name=name)
    except TypeError:
        return None
    if hasattr(material, "color"):
        material.color = rgba
    elif hasattr(material, "rgba"):
        material.rgba = rgba
    return material


def _visual(
    part,
    geometry,
    *,
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    material: Material | None = None,
    name: str | None = None,
) -> None:
    kwargs = {"origin": Origin(xyz=xyz, rpy=rpy)}
    if material is not None:
        kwargs["material"] = material
    if name is not None:
        kwargs["name"] = name
    part.visual(geometry, **kwargs)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pet_door_flap", assets=ASSETS)

    materials = {
        "frame_white": _make_material("frame_white", (0.94, 0.94, 0.91, 1.0)),
        "frame_gray": _make_material("frame_gray", (0.78, 0.79, 0.80, 1.0)),
        "charcoal_plastic": _make_material("charcoal_plastic", (0.18, 0.18, 0.19, 1.0)),
        "rubber_black": _make_material("rubber_black", (0.08, 0.08, 0.08, 1.0)),
        "smoked_panel": _make_material("smoked_panel", (0.34, 0.41, 0.46, 0.34)),
        "steel": _make_material("steel", (0.72, 0.74, 0.76, 1.0)),
    }
    for material in materials.values():
        if material is not None:
            model.materials.append(material)

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.048, 0.292, 0.390)),
        mass=1.35,
        origin=Origin(xyz=(0.003, 0.0, 0.0)),
    )

    # Front trim ring.
    _visual(
        frame,
        Box((0.016, 0.292, 0.074)),
        xyz=(0.018, 0.0, 0.158),
        material=materials["frame_white"],
        name="front_top_bezel",
    )
    _visual(
        frame,
        Box((0.016, 0.041, 0.268)),
        xyz=(0.018, -0.1255, -0.012),
        material=materials["frame_white"],
        name="front_left_bezel",
    )
    _visual(
        frame,
        Box((0.016, 0.041, 0.268)),
        xyz=(0.018, 0.1255, -0.012),
        material=materials["frame_white"],
        name="front_right_bezel",
    )
    _visual(
        frame,
        Box((0.018, 0.292, 0.056)),
        xyz=(0.017, 0.0, -0.167),
        material=materials["frame_white"],
        name="front_bottom_bezel",
    )
    _visual(
        frame,
        Box((0.022, 0.226, 0.024)),
        xyz=(0.015, 0.0, 0.144),
        material=materials["frame_gray"],
        name="front_hinge_cover",
    )

    # Rear trim ring for the opposite face of the door or wall.
    _visual(
        frame,
        Box((0.012, 0.278, 0.062)),
        xyz=(-0.015, 0.0, 0.159),
        material=materials["frame_gray"],
        name="rear_top_bezel",
    )
    _visual(
        frame,
        Box((0.012, 0.036, 0.308)),
        xyz=(-0.015, -0.121, -0.013),
        material=materials["frame_gray"],
        name="rear_left_bezel",
    )
    _visual(
        frame,
        Box((0.012, 0.036, 0.308)),
        xyz=(-0.015, 0.121, -0.013),
        material=materials["frame_gray"],
        name="rear_right_bezel",
    )
    _visual(
        frame,
        Box((0.012, 0.278, 0.050)),
        xyz=(-0.015, 0.0, -0.170),
        material=materials["frame_gray"],
        name="rear_bottom_bezel",
    )

    # Through-wall or through-door tunnel liner.
    _visual(
        frame,
        Box((0.026, 0.010, 0.262)),
        xyz=(0.0, -0.108, -0.008),
        material=materials["charcoal_plastic"],
        name="left_tunnel_liner",
    )
    _visual(
        frame,
        Box((0.026, 0.010, 0.262)),
        xyz=(0.0, 0.108, -0.008),
        material=materials["charcoal_plastic"],
        name="right_tunnel_liner",
    )
    _visual(
        frame,
        Box((0.026, 0.214, 0.014)),
        xyz=(0.0, 0.0, -0.142),
        material=materials["charcoal_plastic"],
        name="bottom_tunnel_liner",
    )
    _visual(
        frame,
        Box((0.008, 0.010, 0.014)),
        xyz=(0.013, -0.106, 0.122),
        material=materials["charcoal_plastic"],
        name="front_left_hinge_boss",
    )
    _visual(
        frame,
        Box((0.008, 0.010, 0.014)),
        xyz=(0.013, 0.106, 0.122),
        material=materials["charcoal_plastic"],
        name="front_right_hinge_boss",
    )
    _visual(
        frame,
        Box((0.008, 0.010, 0.014)),
        xyz=(-0.013, -0.104, 0.122),
        material=materials["charcoal_plastic"],
        name="rear_left_hinge_boss",
    )
    _visual(
        frame,
        Box((0.008, 0.010, 0.014)),
        xyz=(-0.013, 0.104, 0.122),
        material=materials["charcoal_plastic"],
        name="rear_right_hinge_boss",
    )

    # Mounting screw covers.
    for y, z, name in (
        (-0.108, 0.132, "front_upper_left_cap"),
        (0.108, 0.132, "front_upper_right_cap"),
        (-0.108, -0.154, "front_lower_left_cap"),
        (0.108, -0.154, "front_lower_right_cap"),
    ):
        _visual(
            frame,
            Cylinder(radius=0.008, length=0.008),
            xyz=(0.023, y, z),
            rpy=(0.0, pi / 2.0, 0.0),
            material=materials["frame_white"],
            name=name,
        )
    for y, z, name in (
        (-0.100, 0.126, "rear_upper_left_cap"),
        (0.100, 0.126, "rear_upper_right_cap"),
        (-0.100, -0.150, "rear_lower_left_cap"),
        (0.100, -0.150, "rear_lower_right_cap"),
    ):
        _visual(
            frame,
            Cylinder(radius=0.006, length=0.006),
            xyz=(-0.019, y, z),
            rpy=(0.0, pi / 2.0, 0.0),
            material=materials["frame_gray"],
            name=name,
        )

    flap = model.part("flap")
    flap.inertial = Inertial.from_geometry(
        Box((0.012, 0.198, 0.250)),
        mass=0.42,
        origin=Origin(xyz=(0.001, 0.0, -0.120)),
    )

    _visual(
        flap,
        Cylinder(radius=0.006, length=0.176),
        xyz=(-0.001, 0.0, 0.0),
        rpy=(pi / 2.0, 0.0, 0.0),
        material=materials["charcoal_plastic"],
        name="hinge_barrel",
    )
    _visual(
        flap,
        Box((0.008, 0.186, 0.014)),
        xyz=(0.001, 0.0, -0.010),
        material=materials["charcoal_plastic"],
        name="top_trim",
    )
    _visual(
        flap,
        Box((0.004, 0.174, 0.214)),
        xyz=(0.0, 0.0, -0.121),
        material=materials["smoked_panel"],
        name="clear_panel",
    )
    _visual(
        flap,
        Box((0.008, 0.009, 0.228)),
        xyz=(0.002, -0.094, -0.121),
        material=materials["rubber_black"],
        name="left_trim",
    )
    _visual(
        flap,
        Box((0.008, 0.009, 0.228)),
        xyz=(0.002, 0.094, -0.121),
        material=materials["rubber_black"],
        name="right_trim",
    )
    _visual(
        flap,
        Box((0.010, 0.168, 0.013)),
        xyz=(0.003, 0.0, -0.233),
        material=materials["rubber_black"],
        name="bottom_seal_bar",
    )
    _visual(
        flap,
        Box((0.006, 0.150, 0.004)),
        xyz=(0.004, 0.0, -0.233),
        material=materials["steel"],
        name="weighted_insert",
    )
    _visual(
        flap,
        Box((0.008, 0.024, 0.018)),
        xyz=(0.001, -0.060, -0.004),
        material=materials["charcoal_plastic"],
        name="left_hinge_knuckle",
    )
    _visual(
        flap,
        Box((0.008, 0.024, 0.018)),
        xyz=(0.001, 0.060, -0.004),
        material=materials["charcoal_plastic"],
        name="right_hinge_knuckle",
    )

    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent="frame",
        child="flap",
        origin=Origin(xyz=(0.0, 0.0, 0.122)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=-0.65,
            upper=0.65,
        ),
    )

    return model


def _aabb_bounds(aabb) -> tuple[float, float, float, float, float, float]:
    if aabb is None:
        raise AssertionError("Expected part geometry to produce an AABB, but got None.")
    if isinstance(aabb, (tuple, list)):
        if len(aabb) == 6:
            paired_order = (aabb[0], aabb[1], aabb[2], aabb[3], aabb[4], aabb[5])
            flattened_min_max = (aabb[0], aabb[3], aabb[1], aabb[4], aabb[2], aabb[5])

            def _is_valid(bounds: tuple[float, float, float, float, float, float]) -> bool:
                return bounds[0] <= bounds[1] and bounds[2] <= bounds[3] and bounds[4] <= bounds[5]

            def _volume(bounds: tuple[float, float, float, float, float, float]) -> float:
                return (bounds[1] - bounds[0]) * (bounds[3] - bounds[2]) * (bounds[5] - bounds[4])

            paired_valid = _is_valid(paired_order)
            flattened_valid = _is_valid(flattened_min_max)
            if paired_valid and not flattened_valid:
                return paired_order
            if flattened_valid and not paired_valid:
                return flattened_min_max
            if paired_valid and flattened_valid:
                return (
                    paired_order
                    if _volume(paired_order) <= _volume(flattened_min_max)
                    else flattened_min_max
                )
        if len(aabb) == 2 and all(isinstance(v, (tuple, list)) and len(v) == 3 for v in aabb):
            return aabb[0][0], aabb[1][0], aabb[0][1], aabb[1][1], aabb[0][2], aabb[1][2]
    if all(hasattr(aabb, attr) for attr in ("min_x", "max_x", "min_y", "max_y", "min_z", "max_z")):
        return aabb.min_x, aabb.max_x, aabb.min_y, aabb.max_y, aabb.min_z, aabb.max_z
    if hasattr(aabb, "min") and hasattr(aabb, "max"):
        return aabb.min[0], aabb.max[0], aabb.min[1], aabb.max[1], aabb.min[2], aabb.max[2]
    if hasattr(aabb, "minimum") and hasattr(aabb, "maximum"):
        return (
            aabb.minimum[0],
            aabb.maximum[0],
            aabb.minimum[1],
            aabb.maximum[1],
            aabb.minimum[2],
            aabb.maximum[2],
        )
    if hasattr(aabb, "mins") and hasattr(aabb, "maxs"):
        return aabb.mins[0], aabb.maxs[0], aabb.mins[1], aabb.maxs[1], aabb.mins[2], aabb.maxs[2]
    raise AssertionError(f"Unsupported AABB representation: {type(aabb)!r}")


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def _check_flap_pose(ctx: TestContext, label: str, x_behavior: str, top_behavior: str) -> None:
    frame_bounds = _aabb_bounds(ctx.part_world_aabb("frame"))
    flap_bounds = _aabb_bounds(ctx.part_world_aabb("flap"))
    frame_min_x, frame_max_x, frame_min_y, frame_max_y, frame_min_z, frame_max_z = frame_bounds
    flap_min_x, flap_max_x, flap_min_y, flap_max_y, flap_min_z, flap_max_z = flap_bounds

    _require(
        flap_min_y > frame_min_y + 0.035,
        f"{label}: flap should stay well inside the left side of the frame opening.",
    )
    _require(
        flap_max_y < frame_max_y - 0.035,
        f"{label}: flap should stay well inside the right side of the frame opening.",
    )
    _require(
        flap_min_z > frame_min_z + 0.050,
        f"{label}: flap bottom should stay meaningfully above the frame's lower outer edge.",
    )

    if top_behavior == "tucked":
        _require(
            0.118 <= flap_max_z <= 0.130,
            f"{label}: hinge barrel should remain tucked just below the top of the opening.",
        )
    elif top_behavior == "within_frame":
        _require(
            frame_max_z - 0.140 <= flap_max_z <= frame_max_z - 0.006,
            f"{label}: swung flap should still terminate below the outer top trim instead of floating away.",
        )
    else:
        raise AssertionError(f"Unexpected top behavior {top_behavior!r}")

    if x_behavior == "centered":
        _require(
            flap_min_x > frame_min_x + 0.010,
            f"{label}: rest pose should keep the flap inside the frame depth envelope.",
        )
        _require(
            flap_max_x < frame_max_x - 0.010,
            f"{label}: rest pose should keep the flap inside the frame depth envelope.",
        )
    elif x_behavior == "positive":
        _require(
            flap_max_x > frame_max_x + 0.035,
            f"{label}: swung pose should project clearly out past the front face.",
        )
    elif x_behavior == "negative":
        _require(
            flap_min_x < frame_min_x - 0.035,
            f"{label}: swung pose should project clearly out past the rear face.",
        )
    else:
        raise AssertionError(f"Unexpected x behavior {x_behavior!r}")


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.0025, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("flap", "frame", axes="xy", min_overlap=0.010)
    _check_flap_pose(ctx, "rest pose", x_behavior="centered", top_behavior="tucked")

    with ctx.pose(frame_to_flap=0.45):
        ctx.expect_aabb_overlap("flap", "frame", axes="xy", min_overlap=0.010)
        _check_flap_pose(
            ctx, "mid swing rearward", x_behavior="negative", top_behavior="within_frame"
        )

    with ctx.pose(frame_to_flap=-0.45):
        ctx.expect_aabb_overlap("flap", "frame", axes="xy", min_overlap=0.010)
        _check_flap_pose(
            ctx, "mid swing forward", x_behavior="positive", top_behavior="within_frame"
        )

    with ctx.pose(frame_to_flap=0.60):
        ctx.expect_aabb_overlap("flap", "frame", axes="xy", min_overlap=0.010)
        _check_flap_pose(
            ctx,
            "near-limit rearward swing",
            x_behavior="negative",
            top_behavior="within_frame",
        )

    with ctx.pose(frame_to_flap=-0.60):
        ctx.expect_aabb_overlap("flap", "frame", axes="xy", min_overlap=0.010)
        _check_flap_pose(
            ctx,
            "near-limit forward swing",
            x_behavior="positive",
            top_behavior="within_frame",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
