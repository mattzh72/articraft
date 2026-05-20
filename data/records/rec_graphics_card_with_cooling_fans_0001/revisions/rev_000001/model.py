from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    BoxGeometry,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)


def _circle_profile(
    radius: float,
    *,
    cx: float = 0.0,
    cy: float = 0.0,
    segments: int = 32,
) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _translate_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _build_shroud_face_mesh():
    outer_profile = [
        (-0.144, -0.054),
        (0.114, -0.054),
        (0.131, -0.049),
        (0.142, -0.039),
        (0.145, -0.024),
        (0.145, 0.024),
        (0.142, 0.039),
        (0.131, 0.049),
        (0.114, 0.054),
        (-0.137, 0.054),
        (-0.144, 0.046),
    ]
    holes = [
        _circle_profile(0.039, cx=-0.091, cy=0.0, segments=28),
        _circle_profile(0.039, cx=0.0, cy=0.0, segments=28),
        _circle_profile(0.039, cx=0.091, cy=0.0, segments=28),
    ]
    face_geom = ExtrudeWithHolesGeometry(
        outer_profile,
        holes,
        height=0.006,
        cap=True,
        center=True,
        closed=True,
    )
    return mesh_from_geometry(face_geom, ASSETS.mesh_dir / "graphics_card_shroud_face.obj")


def _build_fan_rotor_mesh(name: str):
    rim_geom = LatheGeometry(
        [
            (0.0325, -0.0022),
            (0.0360, -0.0022),
            (0.0360, 0.0022),
            (0.0325, 0.0022),
        ],
        segments=56,
    )
    blade_geom = BoxGeometry((0.024, 0.008, 0.0024))
    blade_geom.translate(0.021, 0.0, 0.0)
    blade_geom.rotate_z(math.radians(24.0))
    for idx in range(9):
        rim_geom.merge(blade_geom.clone().rotate_z(idx * (2.0 * math.pi / 9.0)))
    return mesh_from_geometry(rim_geom, ASSETS.mesh_dir / name)


def _build_bracket_mesh():
    outer_profile = [
        (0.022, -0.059),
        (0.022, 0.048),
        (0.018, 0.056),
        (0.008, 0.064),
        (-0.008, 0.064),
        (-0.018, 0.056),
        (-0.022, 0.048),
        (-0.022, -0.059),
    ]
    port_profile = rounded_rect_profile(0.013, 0.023, radius=0.0018, corner_segments=4)
    holes = [
        _translate_profile(port_profile, dx=0.0, dy=-0.021),
        _translate_profile(port_profile, dx=0.0, dy=0.010),
        _circle_profile(0.0045, cx=0.0, cy=0.056, segments=20),
    ]
    bracket_geom = ExtrudeWithHolesGeometry(
        outer_profile,
        holes,
        height=0.002,
        cap=True,
        center=True,
        closed=True,
    )
    return mesh_from_geometry(bracket_geom, ASSETS.mesh_dir / "graphics_card_bracket.obj")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="graphics_card", assets=ASSETS)

    polymer_black = model.material("polymer_black", rgba=(0.08, 0.09, 0.10, 1.0))
    satin_black = model.material("satin_black", rgba=(0.14, 0.15, 0.17, 1.0))
    dark_aluminum = model.material("dark_aluminum", rgba=(0.34, 0.36, 0.39, 1.0))
    fin_aluminum = model.material("fin_aluminum", rgba=(0.56, 0.58, 0.60, 1.0))
    bracket_steel = model.material("bracket_steel", rgba=(0.67, 0.69, 0.72, 1.0))
    pcb_green = model.material("pcb_green", rgba=(0.06, 0.18, 0.10, 1.0))
    contact_gold = model.material("contact_gold", rgba=(0.83, 0.68, 0.28, 1.0))

    shroud_face_mesh = _build_shroud_face_mesh()
    fan_rotor_mesh = _build_fan_rotor_mesh("graphics_card_fan_rotor.obj")
    bracket_mesh = _build_bracket_mesh()

    body = model.part("gpu_body")
    front_plate = body.visual(
        shroud_face_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=polymer_black,
        name="front_plate",
    )
    body.visual(
        Box((0.275, 0.010, 0.044)),
        origin=Origin(xyz=(0.000, 0.051, -0.001)),
        material=satin_black,
        name="top_rail",
    )
    body.visual(
        Box((0.275, 0.010, 0.044)),
        origin=Origin(xyz=(0.000, -0.051, -0.001)),
        material=satin_black,
        name="bottom_rail",
    )
    rear_spine = body.visual(
        Box((0.008, 0.108, 0.044)),
        origin=Origin(xyz=(-0.139, 0.0, -0.001)),
        material=satin_black,
        name="rear_spine",
    )
    body.visual(
        Box((0.018, 0.098, 0.044)),
        origin=Origin(xyz=(0.136, 0.0, -0.001)),
        material=satin_black,
        name="nose_cap",
    )
    heatsink = body.visual(
        Box((0.278, 0.104, 0.018)),
        origin=Origin(xyz=(0.000, 0.0, -0.008)),
        material=fin_aluminum,
        name="heatsink_core",
    )
    body.visual(
        Box((0.286, 0.108, 0.002)),
        origin=Origin(xyz=(0.000, 0.0, -0.022)),
        material=dark_aluminum,
        name="backplate",
    )
    body.visual(
        Box((0.282, 0.098, 0.004)),
        origin=Origin(xyz=(0.000, 0.0, -0.019)),
        material=pcb_green,
        name="pcb",
    )
    body.visual(
        Box((0.088, 0.010, 0.002)),
        origin=Origin(xyz=(-0.010, -0.049, -0.019)),
        material=contact_gold,
        name="pcie_fingers",
    )
    body.visual(
        Box((0.024, 0.012, 0.012)),
        origin=Origin(xyz=(0.055, 0.056, -0.004)),
        material=satin_black,
        name="power_connector",
    )
    body.visual(
        Box((0.060, 0.008, 0.004)),
        origin=Origin(xyz=(0.030, 0.053, 0.007)),
        material=dark_aluminum,
        name="top_badge_bar",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.290, 0.118, 0.046)),
        mass=1.25,
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
    )

    bracket = model.part("io_bracket")
    bracket_plate = bracket.visual(
        bracket_mesh,
        origin=Origin(xyz=(-0.001, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bracket_steel,
        name="bracket_plate",
    )
    bracket.visual(
        Box((0.008, 0.016, 0.006)),
        origin=Origin(xyz=(-0.004, 0.056, 0.0)),
        material=bracket_steel,
        name="mount_tab",
    )
    bracket.inertial = Inertial.from_geometry(
        Box((0.008, 0.125, 0.050)),
        mass=0.10,
        origin=Origin(xyz=(-0.002, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_bracket",
        ArticulationType.FIXED,
        parent=body,
        child=bracket,
        origin=Origin(xyz=(-0.143, 0.0, 0.0)),
    )

    fan_centers = (-0.091, 0.0, 0.091)
    fan_parts = []
    fan_rotor_visuals = []
    fan_hub_visuals = []
    fan_joints = []
    for idx, x_pos in enumerate(fan_centers, start=1):
        fan = model.part(f"fan_{idx}")
        rotor = fan.visual(
            fan_rotor_mesh,
            origin=Origin(),
            material=satin_black,
            name=f"fan_{idx}_rotor",
        )
        hub = fan.visual(
            Cylinder(radius=0.0105, length=0.006),
            origin=Origin(),
            material=dark_aluminum,
            name=f"fan_{idx}_hub",
        )
        fan.inertial = Inertial.from_geometry(
            Cylinder(radius=0.036, length=0.008),
            mass=0.05,
            origin=Origin(),
        )
        joint = model.articulation(
            f"body_to_fan_{idx}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=fan,
            origin=Origin(xyz=(x_pos, 0.0, 0.010)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.4, velocity=45.0),
        )
        fan_parts.append(fan)
        fan_rotor_visuals.append(rotor)
        fan_hub_visuals.append(hub)
        fan_joints.append(joint)

    model.meta["test_refs"] = {
        "body": body,
        "bracket": bracket,
        "bracket_plate": bracket_plate,
        "front_plate": front_plate,
        "rear_spine": rear_spine,
        "heatsink": heatsink,
        "fans": fan_parts,
        "fan_rotors": fan_rotor_visuals,
        "fan_hubs": fan_hub_visuals,
        "fan_joints": fan_joints,
    }
    return model


def run_tests() -> TestReport:
    refs = object_model.meta["test_refs"]
    body = refs["body"]
    bracket = refs["bracket"]
    bracket_plate = refs["bracket_plate"]
    front_plate = refs["front_plate"]
    rear_spine = refs["rear_spine"]
    heatsink = refs["heatsink"]
    fans = refs["fans"]
    fan_rotors = refs["fan_rotors"]
    fan_joints = refs["fan_joints"]

    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_articulation_origin_far_from_geometry(tol=0.015)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.fail_if_articulation_overlaps(max_pose_samples=128)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    # Add narrow allowances here when conservative QC reports acceptable cases.
    # Add prompt-specific expect_* semantic checks below; they are the main regressions.
    ctx.expect_aabb_contact(body, bracket)
    ctx.expect_aabb_gap(
        body,
        bracket,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0002,
        positive_elem=rear_spine,
        negative_elem=bracket_plate,
        name="bracket seats flush to card body",
    )
    ctx.expect_aabb_overlap(body, bracket, axes="yz", min_overlap=0.035)
    ctx.expect_aabb_gap(
        fans[1],
        fans[0],
        axis="x",
        min_gap=0.012,
        max_gap=0.028,
        positive_elem=fan_rotors[1],
        negative_elem=fan_rotors[0],
        name="left and center cooling fans are clearly separated",
    )
    ctx.expect_aabb_gap(
        fans[2],
        fans[1],
        axis="x",
        min_gap=0.012,
        max_gap=0.028,
        positive_elem=fan_rotors[2],
        negative_elem=fan_rotors[1],
        name="center and right cooling fans are clearly separated",
    )
    ctx.expect_aabb_overlap(fans[0], fans[1], axes="y", min_overlap=0.068)
    ctx.expect_aabb_overlap(fans[1], fans[2], axes="y", min_overlap=0.068)
    ctx.expect_origin_distance(fans[0], fans[1], axes="z", max_dist=0.001)
    ctx.expect_origin_distance(fans[1], fans[2], axes="z", max_dist=0.001)

    for fan, rotor in zip(fans, fan_rotors):
        ctx.expect_aabb_overlap(body, fan, axes="xy", min_overlap=0.070)
        ctx.expect_aabb_gap(
            body,
            fan,
            axis="z",
            max_gap=0.007,
            max_penetration=0.0,
            positive_elem=front_plate,
            negative_elem=rotor,
            name=f"{fan.name} sits just behind shroud opening",
        )
        ctx.expect_aabb_gap(
            fan,
            body,
            axis="z",
            min_gap=0.004,
            positive_elem=rotor,
            negative_elem=heatsink,
            name=f"{fan.name} clears heatsink core",
        )

    with ctx.pose(
        {
            fan_joints[0]: math.pi / 7.0,
            fan_joints[1]: math.pi / 5.0,
            fan_joints[2]: math.pi / 3.0,
        }
    ):
        for fan, rotor in zip(fans, fan_rotors):
            ctx.expect_aabb_overlap(body, fan, axes="xy", min_overlap=0.068)
            ctx.expect_aabb_gap(
                body,
                fan,
                axis="z",
                max_gap=0.008,
                max_penetration=0.0,
                positive_elem=front_plate,
                negative_elem=rotor,
                name=f"{fan.name} remains seated behind opening in spun pose",
            )

    with ctx.pose(
        {
            fan_joints[0]: 1.9,
            fan_joints[1]: 2.6,
            fan_joints[2]: 0.9,
        }
    ):
        for fan, rotor in zip(fans, fan_rotors):
            ctx.expect_aabb_gap(
                body,
                fan,
                axis="z",
                max_gap=0.008,
                max_penetration=0.0,
                positive_elem=front_plate,
                negative_elem=rotor,
                name=f"{fan.name} stays behind front plate at alternate spin phase",
            )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
