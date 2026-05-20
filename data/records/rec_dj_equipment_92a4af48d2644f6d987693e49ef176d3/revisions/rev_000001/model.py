from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    Material,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    mesh_from_geometry,
    mesh_from_cadquery,
)
import cadquery as cq

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_controller")

    # Base chassis
    base = model.part("base")
    
    # Base dimensions
    b_w, b_d, b_h = 0.60, 0.35, 0.05
    
    # Buttons
    button_defs = [
        ("cue_0", -0.26, -0.12, (0.8, 0.2, 0.2)),
        ("play_0", -0.22, -0.12, (0.2, 0.8, 0.2)),
        ("cue_1", 0.10, -0.12, (0.8, 0.2, 0.2)),
        ("play_1", 0.14, -0.12, (0.2, 0.8, 0.2)),
    ]

    # Knobs
    knob_defs = [
        ("eq_0_high", -0.04, 0.14),
        ("eq_0_mid", -0.04, 0.10),
        ("eq_0_low", -0.04, 0.06),
        ("eq_1_high", 0.04, 0.14),
        ("eq_1_mid", 0.04, 0.10),
        ("eq_1_low", 0.04, 0.06),
    ]

    base_cq = (
        cq.Workplane("XY")
        .box(b_w, b_d, b_h)
        .edges("|Z").fillet(0.01)
        .edges(">Z").chamfer(0.002)
    )

    top_wp = base_cq.workplane(offset=b_h / 2)

    # Slots
    v_slots = [
        (-0.26, 0.0, 0.008, 0.10),   # Pitch L
        (0.26, 0.0, 0.008, 0.10),    # Pitch R
        (-0.04, -0.02, 0.008, 0.08), # Ch0 fader
        (0.04, -0.02, 0.008, 0.08),  # Ch1 fader
    ]
    for x, y, w, h in v_slots:
        top_wp = top_wp.pushPoints([(x, y)]).rect(w, h).cutBlind(-0.01)

    # Crossfader slot
    top_wp = top_wp.pushPoints([(0.0, -0.12)]).rect(0.08, 0.008).cutBlind(-0.01)

    # Platter recesses
    top_wp = top_wp.pushPoints([(-0.18, 0.04), (0.18, 0.04)]).circle(0.072).cutBlind(-0.005)

    # Button holes
    btn_pts = [(x, y) for _, x, y, _ in button_defs]
    top_wp = top_wp.pushPoints(btn_pts).circle(0.011).cutBlind(-0.005)

    # Knob holes
    knob_pts = [(x, y) for _, x, y in knob_defs]
    top_wp = top_wp.pushPoints(knob_pts).circle(0.005).cutBlind(-0.005)

    # Master knob hole
    top_wp = top_wp.pushPoints([(0.0, 0.14)]).circle(0.005).cutBlind(-0.005)

    base_cq = top_wp

    base.visual(
        mesh_from_cadquery(base_cq, "base_shell"),
        origin=Origin(),
        material=Material("base_mat", color=(0.15, 0.15, 0.15)),
        name="base_vis"
    )

    base_top_z = b_h / 2

    # Platters
    for i, x_pos in enumerate([-0.18, 0.18]):
        platter = model.part(f"platter_{i}")
        platter.visual(
            Cylinder(0.07, 0.015),
            origin=Origin(xyz=(0.0, 0.0, 0.0075)),
            material=Material(f"platter_mat_{i}", color=(0.8, 0.8, 0.8)),
            name=f"platter_{i}_vis"
        )
        model.articulation(
            f"platter_{i}_spin",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=platter,
            origin=Origin(xyz=(x_pos, 0.04, base_top_z - 0.005)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.0, velocity=10.0)
        )

    # Pitch Faders
    for i, x_pos in enumerate([-0.26, 0.26]):
        fader = model.part(f"pitch_fader_{i}")
        fader.visual(
            Box((0.014, 0.024, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, 0.007)),
            material=Material(f"pitch_fader_mat_{i}", color=(0.05, 0.05, 0.05)),
            name=f"pitch_fader_{i}_vis"
        )
        # Add a small stem to connect to the slot
        fader.visual(
            Box((0.004, 0.004, 0.01)),
            origin=Origin(xyz=(0.0, 0.0, -0.005)),
            material=Material(f"pitch_fader_stem_mat_{i}", color=(0.5, 0.5, 0.5)),
            name=f"pitch_fader_{i}_stem"
        )
        model.articulation(
            f"pitch_fader_{i}_slide",
            ArticulationType.PRISMATIC,
            parent=base,
            child=fader,
            origin=Origin(xyz=(x_pos, 0.0, base_top_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=-0.035, upper=0.035)
        )

    # Channel Faders
    for i, x_pos in enumerate([-0.04, 0.04]):
        fader = model.part(f"channel_fader_{i}")
        fader.visual(
            Box((0.014, 0.024, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, 0.007)),
            material=Material(f"channel_fader_mat_{i}", color=(0.05, 0.05, 0.05)),
            name=f"channel_fader_{i}_vis"
        )
        fader.visual(
            Box((0.004, 0.004, 0.01)),
            origin=Origin(xyz=(0.0, 0.0, -0.005)),
            material=Material(f"channel_fader_stem_mat_{i}", color=(0.5, 0.5, 0.5)),
            name=f"channel_fader_{i}_stem"
        )
        model.articulation(
            f"channel_fader_{i}_slide",
            ArticulationType.PRISMATIC,
            parent=base,
            child=fader,
            origin=Origin(xyz=(x_pos, -0.02, base_top_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=-0.025, upper=0.025)
        )

    # Crossfader
    cf = model.part("crossfader")
    cf.visual(
        Box((0.024, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=Material("crossfader_mat", color=(0.05, 0.05, 0.05)),
        name="crossfader_vis"
    )
    cf.visual(
        Box((0.004, 0.004, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=Material("crossfader_stem_mat", color=(0.5, 0.5, 0.5)),
        name="crossfader_stem"
    )
    model.articulation(
        "crossfader_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=cf,
        origin=Origin(xyz=(0.0, -0.12, base_top_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=-0.025, upper=0.025)
    )

    # Buttons
    for name, x, y, col in button_defs:
        btn = model.part(name)
        btn.visual(
            Cylinder(0.01, 0.008),
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
            material=Material(f"{name}_mat", color=col),
            name=f"{name}_vis"
        )
        model.articulation(
            f"{name}_press",
            ArticulationType.PRISMATIC,
            parent=base,
            child=btn,
            origin=Origin(xyz=(x, y, base_top_z - 0.004)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=1.0, lower=0.0, upper=0.003)
        )

    # Knobs
    eq_knob_geo = KnobGeometry(
        0.016, 0.016,
        body_style="cylindrical",
        grip=KnobGrip(style="ribbed", count=18, depth=0.0005, width=0.001),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0005),
        center=False,
    )
    eq_mesh = mesh_from_geometry(eq_knob_geo, "eq_knob_mesh")

    for name, x, y in knob_defs:
        knob = model.part(name)
        knob.visual(
            eq_mesh,
            origin=Origin(),
            material=Material(f"{name}_mat", color=(0.2, 0.2, 0.2)),
            name=f"{name}_vis"
        )
        knob.visual(
            Cylinder(0.004, 0.004),
            origin=Origin(xyz=(0.0, 0.0, -0.002)),
            material=Material(f"{name}_stem_mat", color=(0.1, 0.1, 0.1)),
            name=f"{name}_stem"
        )
        model.articulation(
            f"{name}_turn",
            ArticulationType.REVOLUTE,
            parent=base,
            child=knob,
            origin=Origin(xyz=(x, y, base_top_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.5, velocity=5.0, lower=-2.5, upper=2.5)
        )

    # Master Knob
    master_knob_geo = KnobGeometry(
        0.024, 0.018,
        body_style="faceted",
        top_diameter=0.020,
        grip=KnobGrip(style="fluted", count=12, depth=0.001),
        indicator=KnobIndicator(style="wedge", mode="raised"),
        center=False,
    )
    master_mesh = mesh_from_geometry(master_knob_geo, "master_knob_mesh")
    mk = model.part("master_knob")
    mk.visual(
        master_mesh,
        origin=Origin(),
        material=Material("master_knob_mat", color=(0.8, 0.1, 0.1)),
        name="master_knob_vis"
    )
    mk.visual(
        Cylinder(0.004, 0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=Material("master_knob_stem_mat", color=(0.1, 0.1, 0.1)),
        name="master_knob_stem"
    )
    model.articulation(
        "master_knob_turn",
        ArticulationType.REVOLUTE,
        parent=base,
        child=mk,
        origin=Origin(xyz=(0.0, 0.14, base_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=5.0, lower=-2.5, upper=2.5)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    # Allowances for stems passing through slots
    base = object_model.get_part("base")
    for i in range(2):
        ctx.allow_overlap(
            object_model.get_part(f"platter_{i}"), base,
            reason="Platter sits in a shallow recess in the base."
        )
        ctx.expect_gap(
            object_model.get_part(f"platter_{i}"), base,
            axis="z", max_penetration=0.006,
            name=f"platter_{i} seating"
        )
        ctx.allow_overlap(
            object_model.get_part(f"pitch_fader_{i}"), base,
            reason="Fader stem passes through slot in base."
        )
        ctx.allow_overlap(
            object_model.get_part(f"channel_fader_{i}"), base,
            reason="Fader stem passes through slot in base."
        )
    ctx.allow_overlap(
        object_model.get_part("crossfader"), base,
        reason="Crossfader stem passes through slot in base."
    )

    return ctx.report()

object_model = build_object_model()